import sys
import os
import math
import time
import numpy as np
from isaacsim import SimulationApp

# --- CONFIG ---
CONFIG = {"headless": False, "install_signal_handlers": False, "width": 1280, "height": 720}
kit = SimulationApp(CONFIG)

import omni
import carb
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, Sdf, PhysxSchema

# --- 0. ROBUST ENVIRONMENT PATCH ---
if "ISAAC_PATH" not in os.environ:
    candidates = [r"C:\isaac-sim", os.environ.get("USERPROFILE", "") + r"\AppData\Local\ov\pkg\isaac_sim-2023.1.1"]
    isaac_path = None
    for c in candidates:
        if os.path.exists(c):
            isaac_path = c
            break
    if isaac_path:
        os.environ["ISAAC_PATH"] = isaac_path
        os.environ["EXP_PATH"] = os.path.join(isaac_path, "apps")
        os.environ["CARB_APP_PATH"] = os.path.join(isaac_path, "kit")
        kit_path = os.path.join(isaac_path, "kit")
        if kit_path not in os.environ["PATH"]:
            os.environ["PATH"] = kit_path + os.pathsep + os.environ["PATH"]
        sys.path.append(os.path.join(kit_path, "kernel", "py"))
        sys.path.append(os.path.join(kit_path, "python", "lib", "site-packages"))
        site_path = os.path.join(isaac_path, "site")
        if site_path not in sys.path:
            sys.path.append(site_path)
        try:
            import sitecustomize
        except ImportError:
            pass

try:
    from isaacsim.core.api.world import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder
    from omni.isaac.core.materials import PreviewSurface
    from omni.physx import get_physx_scene_query_interface
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder
    from omni.isaac.core.materials import PreviewSurface
    from omni.physx import get_physx_scene_query_interface

# Import Gait Config
try:
    from pantin.gait_config import GAIT_PROFILES
except ImportError:
    # Safe fallback if run from root
    sys.path.append(os.getcwd())
    from pantin.gait_config import GAIT_PROFILES

ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

# --- SCENE HELPERS ---

def apply_collision_rigid(prim_path, world):
    stage = world.stage
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid(): return
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)

def create_industrial_stairs(world, position, num_steps=15, step_height=0.15, step_depth=0.25, width=1.0):
    base_pos = np.array(position)
    # Stairs
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        pos = base_pos + np.array([x_offset, 0, z_offset])
        prim_path = f"/World/Environment/Stairs/Step_{i}"
        world.scene.add(VisualCuboid(prim_path=prim_path, name=f"step_{i}", position=pos, scale=np.array([step_depth, width, step_height]), color=np.array([0.3, 0.3, 0.35])))
        apply_collision_rigid(prim_path, world)

    # Catwalk
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([(num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth), 0, (num_steps - 1) * step_height + (step_height / 2.0)])
    cw_path = "/World/Environment/Stairs/Catwalk"
    world.scene.add(VisualCuboid(prim_path=cw_path, name="catwalk", position=catwalk_pos, scale=np.array([catwalk_depth, width, step_height]), color=np.array([0.25, 0.25, 0.3])))
    apply_collision_rigid(cw_path, world)

    # Handrails
    rail_height = 0.9
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2)
    angle_rad = math.atan2(total_rise, total_run)
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height
    rail_offsets_y = [width/2.0, -width/2.0]
    pitch_deg = 90 - math.degrees(angle_rad)
    
    for idx, y_off in enumerate(rail_offsets_y):
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        world.scene.add(VisualCylinder(prim_path=f"/World/Environment/Stairs/Rail_Diag_{idx}", name=f"rail_diag_{idx}", position=rail_pos, scale=np.array([0.025, 0.025, diag_len + 0.5]), color=np.array([0.8, 0.8, 0.2]), orientation=np.array([math.cos(math.radians(pitch_deg)/2), 0, math.sin(math.radians(pitch_deg)/2), 0])))
        
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             world.scene.add(VisualCylinder(prim_path=f"/World/Environment/Stairs/Post_{idx}_{p_idx}", name=f"post_{idx}_{p_idx}", position=post_pos, scale=np.array([0.02, 0.02, rail_height]), color=np.array([0.2, 0.2, 0.2])))

def create_lighting_array(stage, start_pos, count=8, spacing=3.0, height=3.0):
    for i in range(count):
        x = start_pos[0] + i * spacing
        pos = Gf.Vec3f(x, start_pos[1], start_pos[2] + height)
        light_path = f"/World/Lights/Light_{i}"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateIntensityAttr(80000.0) 
        light.CreateRadiusAttr(0.15)
        light.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0)) 
        light.AddTranslateOp().Set(pos)

def create_floor_markings(world, start_pos, end_pos):
    floor_path = "/World/Environment/ConcreteFloor"
    world.scene.add(
        VisualCuboid(
            prim_path=floor_path,
            name="concrete_floor",
            position=np.array([5.0, 0.0, -0.05]), 
            scale=np.array([20.0, 10.0, 0.1]),
            color=np.array([0.2, 0.2, 0.2])
        )
    )
    apply_collision_rigid(floor_path, world)
    
    dist = end_pos[0] - start_pos[0]
    center_x = start_pos[0] + dist / 2.0
    
    path_path = "/World/Environment/BluePath"
    world.scene.add(
        VisualCuboid(
            prim_path=path_path,
            name="blue_path",
            position=np.array([center_x, 0.0, 0.005]), 
            scale=np.array([dist, 0.6, 0.01]), 
            color=np.array([0.0, 0.2, 0.8])
        )
    )

# --- ADVANCED KINEMATICS ---

def solve_leg_ik_analytic(hip_pos, foot_pos, max_knee_deg=120.0):
    L1, L2 = 0.35, 0.35
    vec = foot_pos - hip_pos
    dx = vec[0]
    dz = vec[2]
    dist_sq = dx**2 + dz**2
    dist = math.sqrt(dist_sq)
    dist = min(dist, (L1 + L2) * 0.999)
    dist = max(dist, 0.1)
    
    val = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
    val = max(-1.0, min(1.0, val))
    alpha = math.acos(val)
    knee_angle = math.pi - alpha 
    
    # CLAMP KNEE
    max_knee_rad = math.radians(max_knee_deg)
    knee_angle = max(0.0, min(knee_angle, max_knee_rad))
    # Note: Clamping knee angle effectively means we assume the leg is clearer/straighter.
    # If the physical target was closer, the leg simply won't reach it visually (foot will look detached from target conceptually)
    # But since we set Joint Positions, the robot will render with the clamped leg.
    # This avoids the "Deep Crouch" / 90 degree look.
    
    pitch_vec = math.atan2(dx, -dz)
    val_beta = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    val_beta = max(-1.0, min(1.0, val_beta))
    beta = math.acos(val_beta)
    
    hip_pitch = pitch_vec + beta
    ankle_pitch = -(hip_pitch - knee_angle)
    
    # Final Safety for NaNs
    if math.isnan(hip_pitch): hip_pitch = 0.0
    if math.isnan(knee_angle): knee_angle = 0.0
    if math.isnan(ankle_pitch): ankle_pitch = 0.0
    
    return hip_pitch, knee_angle, ankle_pitch

class SensingWalker:
    def __init__(self, start_pos, stair_start, stair_params):
        self.root_pos = np.array(start_pos)
        self.stair_start = np.array(stair_start)
        self.stair_params = stair_params
        
        self.current_profile_name = "WALK"
        self.profile = GAIT_PROFILES["WALK"]
        
        self.state = 1
        self.t_state = 0.0
        self.t_total = 0.0
        
        self.ds_duration = 0.2
        self.ss_duration = 0.6
        
        # Init from Profile
        self.step_length = self.profile["step_length"]
        self.hip_height = 0.68 # Reduced from 0.72 to prevent hyperextension (0.35+0.35=0.70 max)
        self.foot_sep = 0.2
        
        self.l_foot = self.root_pos + np.array([0, self.foot_sep/2, -self.hip_height])
        self.r_foot = self.root_pos + np.array([0, -self.foot_sep/2, -self.hip_height])
        
        self.swing_start = self.l_foot.copy()
        
        self.physx_query = get_physx_scene_query_interface()
        
        # Determine initial target
        tx = self.r_foot[0] + self.step_length
        tz = self.ray_cast_ground(tx, self.l_foot[1])
        self.swing_end = np.array([tx, self.l_foot[1], tz])
        self.next_swing_leg = 'LEFT' # Current swinging is LEFT
        
    def determine_zone(self, x):
        stairs_begin = self.stair_start[0]
        catwalk_begin = stairs_begin + (self.stair_params[0] * self.stair_params[2])
        
        if x < stairs_begin - 0.1:
            return "WALK"
        elif x >= stairs_begin - 0.1 and x < catwalk_begin:
            # Check if finished climbing
            return "CLIMB"
        else:
            return "WAIT"

    def apply_profile(self, name):
        if name != self.current_profile_name:
            self.current_profile_name = name
            self.profile = GAIT_PROFILES[name]
            # Update params
            self.step_length = self.profile["step_length"]
            # Scale durations?
            if name == "WAIT":
                self.ds_duration = 999.0 # Stay in stance
            elif name == "CLIMB":
                self.ss_duration = 0.8 # Slower climb
            else:
                self.ss_duration = 0.6

    def ray_cast_ground(self, x, y):
        origin = np.array([x, y, 5.0])
        direction = np.array([0.0, 0.0, -1.0])
        dist = 10.0
        try:
             hit = self.physx_query.raycast_closest(origin, direction, dist)
             if hit["hit"]:
                 return hit["position"][2]
        except Exception:
             pass
        return self.get_terrain_height_math(x)

    def get_terrain_height_math(self, x):
        if x < self.stair_start[0]: return 0.0
        rx = x - self.stair_start[0]
        step_idx = int(rx / self.stair_params[0])
        if step_idx < 0: return 0.0
        if step_idx >= self.stair_params[2]: 
            return self.stair_params[2] * self.stair_params[1]
        return (step_idx + 1) * self.stair_params[1]

    def update(self, dt):
        self.t_state += dt
        self.t_total += dt
        joints = {}
        
        # 1. Update Zone/Profile based on Root X
        zone = self.determine_zone(self.root_pos[0])
        self.apply_profile(zone)
        
        if self.current_profile_name == "WAIT":
            # Override State Logic for WAIT
            self.state = 0 # Forced Double Support
            # No steps
        
        # 2. State Machine
        current_dur = self.ss_duration if self.state in [1, 2] else self.ds_duration
        
        if self.t_state >= current_dur and self.current_profile_name != "WAIT":
            self.t_state = 0.0
            
            if self.state == 1: # End L Swing
                self.l_foot = self.swing_end.copy()
                self.state = 0
                self.next_swing_leg = 'RIGHT'
            elif self.state == 2: # End R Swing
                self.r_foot = self.swing_end.copy()
                self.state = 0
                self.next_swing_leg = 'LEFT'
            elif self.state == 0: # End DS
                # LATCH PARAMETERS AT START OF SWING
                self.active_step_len = self.profile["step_length"]
                self.active_swing_height = self.profile.get("swing_height", 0.1)
                
                if self.next_swing_leg == 'LEFT':
                    self.state = 1
                    self.swing_start = self.l_foot.copy()
                    tx = self.r_foot[0] + self.active_step_len
                    tz = self.ray_cast_ground(tx, self.l_foot[1])
                    self.swing_end = np.array([tx, self.l_foot[1], tz])
                else:
                    self.state = 2
                    self.swing_start = self.r_foot.copy()
                    tx = self.l_foot[0] + self.active_step_len
                    tz = self.ray_cast_ground(tx, self.r_foot[1])
                    self.swing_end = np.array([tx, self.r_foot[1], tz])

        # 3. Trajectory
        phase = min(1.0, self.t_state / current_dur)
        if self.current_profile_name != "WAIT":
            if self.state == 1:
                self.l_foot = self.cycloid_interp(self.swing_start, self.swing_end, phase)
            elif self.state == 2:
                self.r_foot = self.cycloid_interp(self.swing_start, self.swing_end, phase)
        
        # 4. Root Update (Anti-Crouch)
        # To prevent "90 degree" bends or "going underground", we must ensure the hips
        # are high enough for the Stance leg to be nearly straight.
        
        # Which leg is stance?
        # If State 1 (Left Swing), Right is Stance.
        # If State 2 (Right Swing), Left is Stance.
        # If State 0 (DS), Both.
        
        stance_z = 0.0
        if self.state == 1: # Left Swing, Right Stance
            stance_z = self.r_foot[2]
        elif self.state == 2: # Right Swing, Left Stance
            stance_z = self.l_foot[2]
        else:
            stance_z = max(self.l_foot[2], self.r_foot[2])
            
        # Drive Root Z from Stance Height
        self.root_pos[0] = (self.l_foot[0] + self.r_foot[0]) / 2.0
        self.root_pos[1] = 0.0 
        self.root_pos[2] = stance_z + self.hip_height 
        
        # Print Debug every 60 frames
        if int(self.t_total * 60) % 60 == 0:
            print(f"[DEBUG] Zone: {zone} | Profile: {self.current_profile_name} | RootZ: {self.root_pos[2]:.2f}")
        
        # 5. IK with Limits
        limit = self.profile["max_knee_deg"]
        
        l_vec = self.l_foot - self.root_pos - np.array([0, 0.07, 0])
        hp, kp, ap = solve_leg_ik_analytic(np.array([0,0,0]), l_vec, limit)
        joints['left_hip_pitch_joint'] = hp
        joints['left_knee_joint'] = min(kp, math.radians(limit))
        joints['left_ankle_pitch_joint'] = ap
        # Explicitly Safe Hip Roll/Yaw
        joints['left_hip_roll_joint'] = 0.05 # Slight abduction to clear pelvis
        joints['left_hip_yaw_joint'] = 0.0
        
        r_vec = self.r_foot - self.root_pos - np.array([0, -0.07, 0])
        hp, kp, ap = solve_leg_ik_analytic(np.array([0,0,0]), r_vec, limit)
        joints['right_hip_pitch_joint'] = hp
        joints['right_knee_joint'] = min(kp, math.radians(limit))
        joints['right_ankle_pitch_joint'] = ap
        # Explicitly Safe Hip Roll/Yaw
        joints['right_hip_roll_joint'] = -0.05
        joints['right_hip_yaw_joint'] = 0.0
        
        # 6. Arms & Head (Safe Pose)
        amp = self.profile["arm_amp"]
        s = math.sin(self.t_total * math.pi * 2.0 / self.profile["cycle_time"]) if self.current_profile_name != "WAIT" else 0.0
        
        # Pitch (Swing)
        joints['left_shoulder_pitch_joint'] = -s * amp
        joints['right_shoulder_pitch_joint'] = s * amp
        
        # Roll (Flare Out to avoid hip collision, but not too much to hit shoulder)
        joints['left_shoulder_roll_joint'] = 0.1
        joints['right_shoulder_roll_joint'] = -0.1
        
        # Elbows & Wrists
        joints['left_elbow_joint'] = 0.5
        joints['right_elbow_joint'] = 0.5
        joints['left_wrist_roll_joint'] = 0.0
        joints['right_wrist_roll_joint'] = 0.0
        
        if self.profile["use_head_look"]:
            # Head Animation
            slow_s = math.sin(self.t_total * 0.5) * 0.5 
            joints['waist_yaw_joint'] = slow_s * 0.2
            pass

        return self.root_pos, joints

    def cycloid_interp(self, start, end, t):
        res = (1-t)*start + t*end
        # Use Latched Swing Height (Safe)
        sh = getattr(self, "active_swing_height", 0.1)
        z_lift = math.sin(t * math.pi) * sh
        base_z = res[2]
        res[2] = max(start[2], end[2]) + z_lift
        return res

def main():
    world = World()
    stage = kit.context.get_stage()
    
    create_lighting_array(stage, start_pos=(-2, 0, 0), count=8, spacing=3.0)
    
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    
    # Enable Self Collisions
    prim = kit.context.get_stage().GetPrimAtPath("/World/G1")
    if prim.IsValid():
        # Check for PhysxArticulationAPI
        # Or UsdPhysics.ArticulationRootAPI
        # Usually it's on the root or base_link.
        # Let's try to find where ArticulationRoot is.
        # G1 USD usually has it on /World/G1
        from pxr import PhysxSchema
        
        # Apply/Get PhysxArticulationAPI
        # Note: In newer Isaac Sim, it might be separate.
        # Let's try setting the property directly if API exists.
        physx_api = PhysxSchema.PhysxArticulationAPI.Get(kit.context.get_stage(), prim.GetPath())
        if not physx_api:
            physx_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            
        physx_api.CreateEnabledSelfCollisionsAttr(True)
        print("[INFO] Robot Self-Collisions ENABLED.")
    
    create_floor_markings(world, start_pos=[0,0,0], end_pos=[3,0,0])
    
    S_NUM = 15; S_H = 0.15; S_D = 0.25
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0], num_steps=S_NUM, step_height=S_H, step_depth=S_D)
    
    walker = SensingWalker(start_pos=[0.1, 0.0, 0.68], stair_start=[3.0, 0.0, 0.0], stair_params=(S_D, S_H, S_NUM))
    
    world.reset()
    
    while kit.is_running():
        world.step(render=True)
        root_pos, joints = walker.update(0.016) 
        
        g1_robot.set_world_pose(position=root_pos, orientation=np.array([1,0,0,0]))
        g1_robot.set_joint_positions(np.array([joints.get(n, 0.0) for n in g1_robot.dof_names]))
        g1_robot.set_joint_velocities(np.zeros(g1_robot.num_dof))

    kit.close()

if __name__ == "__main__":
    main()
