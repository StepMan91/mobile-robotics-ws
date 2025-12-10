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
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, Sdf

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
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder
    from omni.isaac.core.materials import PreviewSurface


ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

# --- SCENE HELPERS ---

def create_industrial_stairs(world, position, num_steps=15, step_height=0.15, step_depth=0.25, width=1.0):
    base_pos = np.array(position)
    # Stairs
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        pos = base_pos + np.array([x_offset, 0, z_offset])
        world.scene.add(VisualCuboid(prim_path=f"/World/Environment/Stairs/Step_{i}", name=f"step_{i}", position=pos, scale=np.array([step_depth, width, step_height]), color=np.array([0.3, 0.3, 0.35])))
    
    # Catwalk
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([(num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth), 0, (num_steps - 1) * step_height + (step_height / 2.0)])
    world.scene.add(VisualCuboid(prim_path="/World/Environment/Stairs/Catwalk", name="catwalk", position=catwalk_pos, scale=np.array([catwalk_depth, width, step_height]), color=np.array([0.25, 0.25, 0.3])))

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
        light.CreateIntensityAttr(80000.0) # Brighter
        light.CreateRadiusAttr(0.15)
        light.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0)) # Cool White Industrial
        light.AddTranslateOp().Set(pos)

def create_floor_markings(world, start_pos, end_pos):
    """
    Creates a concrete floor and a blue path strip.
    """
    # 1. Main Floor (Concrete) - Big Area
    world.scene.add(
        VisualCuboid(
            prim_path="/World/Environment/ConcreteFloor",
            name="concrete_floor",
            position=np.array([5.0, 0.0, -0.05]), # Slightly below zero
            scale=np.array([20.0, 10.0, 0.1]),
            color=np.array([0.4, 0.4, 0.4])
        )
    )
    
    # 2. Blue Path
    # Determine length
    dist = end_pos[0] - start_pos[0]
    center_x = start_pos[0] + dist / 2.0
    
    world.scene.add(
        VisualCuboid(
            prim_path="/World/Environment/BluePath",
            name="blue_path",
            position=np.array([center_x, 0.0, 0.005]), # Just above floor
            scale=np.array([dist, 0.6, 0.01]), # 0.6m Width
            color=np.array([0.0, 0.2, 0.8])
        )
    )

# --- ADVANCED KINEMATICS ---

def solve_leg_ik_analytic(hip_pos, foot_pos):
    """
    Returns (hip_pitch, knee_pitch, ankle_pitch) for G1 leg.
    Lengths: Thigh=0.35, Shin=0.35 (approx).
    """
    L1, L2 = 0.35, 0.35
    
    # Vector Hip->Foot
    vec = foot_pos - hip_pos
    # Local Frame: Hip is origin. Forward=X, Up=Z?
    # No, usually Hip frame Z is down or similar. 
    # Let's work in World Frame component logic relative to Hip.
    
    dx = vec[0] # Forward diff
    dz = vec[2] # Vertical diff (usually negative)
    
    dist_sq = dx**2 + dz**2
    dist = math.sqrt(dist_sq)
    
    # Clamp Reach
    dist = min(dist, (L1 + L2) * 0.999)
    dist = max(dist, 0.1)
    
    # Knee Angle (Law of Cosines)
    # c^2 = a^2 + b^2 - 2ab cos(C) -> dist^2 = L1^2 + L2^2 - 2 L1 L2 cos(pi - knee)
    # cos(pi - knee) = (L1^2 + L2^2 - dist^2) / (2 L1 L2)
    val = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
    val = max(-1.0, min(1.0, val))
    alpha = math.acos(val) # Angle inside triangle opposite to dist
    # Knee bend = pi - alpha? No. Alpha is angle at Knee vertex.
    # True Knee Joint Angle usually deviates from straight (0).
    knee_angle = math.pi - alpha 
    
    # Hip Angle
    # Pitch of the Hip->Foot vector
    # atan2(dz, dx)? 
    # If dx=0, dz=-0.7 -> -pi/2.
    gamma = math.atan2(-dz, dx) # Angle from Horizontal? (Positive down?)
    # Wait, lets use standard atan2(z, x)
    # pitch_vec = atan2(dx, -dz) -> 0 if vertical down. + if forward.
    pitch_vec = math.atan2(dx, -dz)
    
    # Additional angle inside triangle at Hip
    # L2^2 = L1^2 + dist^2 - 2 L1 dist cos(beta)
    val_beta = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    val_beta = max(-1.0, min(1.0, val_beta))
    beta = math.acos(val_beta)
    
    hip_pitch = pitch_vec + beta
    
    # Ankle Pitch
    # We want Foot parallel to ground (Angle 0 global pitch)
    # Sum of angles = hip_pitch - knee_angle + ankle_pitch = 0  (Sign convention varies)
    # If Hip Pitch is positive forward, Knee Pitch positive backward (bend)...
    # G1: Hip Pitch + -> Leg Forward. Knee Pitch + -> Bend.
    # Thigh angle = Hip Pitch.
    # Shin global = Hip Pitch - Knee Pitch.
    # Foot global = Shin global + Ankle Pitch.
    # 0 = Hp - Kp + Ap => Ap = Kp - Hp
    ankle_pitch = -(hip_pitch - knee_angle)
    
    return hip_pitch, knee_angle, ankle_pitch

class ProfessionalWalker:
    def __init__(self, start_pos, stair_start, stair_params):
        self.root_pos = np.array(start_pos)
        self.stair_start = np.array(stair_start)
        self.stair_params = stair_params
        
        # State Machine
        # 0: Double Support (Both feet planted)
        # 1: Left Swing (Left moving, Right planted)
        # 2: Right Swing (Right moving, Left planted)
        self.state = 0 
        self.t_state = 0.0
        
        # Timing
        self.ds_duration = 0.2  # Double support (transfer)
        self.ss_duration = 0.6  # Single support (swing)
        
        # Geometry
        self.step_length = 0.25 # Short, stable steps
        self.hip_height = 0.72 # Constant Hip Height relative to stance foot
        self.foot_sep = 0.2 # Lateral separation
        
        # Feet World Pos
        self.l_foot = self.root_pos + np.array([0, self.foot_sep/2, -self.hip_height])
        self.r_foot = self.root_pos + np.array([0, -self.foot_sep/2, -self.hip_height])
        
        # Trajectory
        self.swing_start = np.zeros(3)
        self.swing_end = np.zeros(3)
        
        # Start walking immediately
        self.state = 1 # Start Left Swing
        self.swing_start = self.l_foot.copy()
        target_x = self.r_foot[0] + self.step_length # Step ahead of right foot
        target_z = self.get_terrain_height(target_x)
        self.swing_end = np.array([target_x, self.l_foot[1], target_z])
        
    def get_terrain_height(self, x):
        if x < self.stair_start[0]: return 0.0
        rx = x - self.stair_start[0]
        step_idx = int(rx / self.stair_params[0])
        if step_idx < 0: return 0.0
        if step_idx >= self.stair_params[2]: 
            # On Catwalk
            return self.stair_params[2] * self.stair_params[1]
        
        # On Step
        return (step_idx + 1) * self.stair_params[1]

    def update(self, dt):
        self.t_state += dt
        
        joints = {}
        
        # --- STATE MACHINE TRANSFORMATIONS ---
        
        # Check Transitions
        current_dur = self.ss_duration if self.state in [1, 2] else self.ds_duration
        
        if self.t_state >= current_dur:
            self.t_state = 0.0
            
            if self.state == 1: # End Left Swing -> Double Support
                self.l_foot = self.swing_end.copy() # Plant
                self.state = 0
                self.next_swing_leg = 'RIGHT'
                
            elif self.state == 2: # End Right Swing -> Double Support
                self.r_foot = self.swing_end.copy() # Plant
                self.state = 0
                self.next_swing_leg = 'LEFT'
                
            elif self.state == 0: # End Double Support -> Swing
                if self.next_swing_leg == 'LEFT':
                    self.state = 1
                    self.swing_start = self.l_foot.copy()
                    # Plan Next Step
                    # Target X = Stance Foot (R) X + Step Length
                    tx = self.r_foot[0] + self.step_length
                    # Stop if end of catwalk
                    if tx > self.stair_start[0] + 5.0: tx = self.r_foot[0] # Stop
                    tz = self.get_terrain_height(tx)
                    self.swing_end = np.array([tx, self.l_foot[1], tz])
                else:
                    self.state = 2
                    self.swing_start = self.r_foot.copy()
                    tx = self.l_foot[0] + self.step_length
                    if tx > self.stair_start[0] + 5.0: tx = self.l_foot[0]
                    tz = self.get_terrain_height(tx)
                    self.swing_end = np.array([tx, self.r_foot[1], tz])

        # --- UPDATE TRAJECTORIES ---
        
        phase = min(1.0, self.t_state / current_dur)
        
        if self.state == 1: # Left Swing
            # Cycloid Swing
            self.l_foot = self.cycloid_interp(self.swing_start, self.swing_end, phase)
        elif self.state == 2: # Right Swing
            self.r_foot = self.cycloid_interp(self.swing_start, self.swing_end, phase)
            
        # --- ROOT UPDATE ---
        # Constant Velocity X approximation
        # Root X is average of feet X?
        # Or interpolate Root X linearly during DS and SS.
        # To avoid rotation:
        # Root Y = 0 (Fixed center path)
        # Root X = (L_Foot_X + R_Foot_X) / 2.0
        # Root Z = Average(Foot_Z) + Hip_Height
        
        # To prevent "flying"/"going under", we CLAMP Root Z to minimum
        
        avg_x = (self.l_foot[0] + self.r_foot[0]) / 2.0
        avg_z = (self.l_foot[2] + self.r_foot[2]) / 2.0
        
        self.root_pos[0] = avg_x
        self.root_pos[1] = 0.0 # Strict Line
        self.root_pos[2] = max(avg_z + self.hip_height, self.hip_height) # Clamp Floor
        
        # --- SOLVE IK ---
        # Left
        l_vec = self.l_foot - self.root_pos - np.array([0, 0.07, 0]) # Hip offset
        hp, kp, ap = solve_leg_ik_analytic(np.array([0,0,0]), l_vec) # Local vector
        joints['left_hip_pitch_joint'] = hp
        joints['left_knee_joint'] = kp
        joints['left_ankle_pitch_joint'] = ap
        
        # Right
        r_vec = self.r_foot - self.root_pos - np.array([0, -0.07, 0])
        hp, kp, ap = solve_leg_ik_analytic(np.array([0,0,0]), r_vec)
        joints['right_hip_pitch_joint'] = hp
        joints['right_knee_joint'] = kp
        joints['right_ankle_pitch_joint'] = ap
        
        # Arms
        # Counter-swing to legs
        # Left Leg Phase (0 if stance, 1 if swing end)
        swing_mag = 0.4
        if self.state == 1: # Left Moving Fwd
            s = math.sin(phase * math.pi)
            # Left Arm moves Back
            l_arm = -s * swing_mag
            r_arm = s * swing_mag
        elif self.state == 2:
            s = math.sin(phase * math.pi)
            l_arm = s * swing_mag
            r_arm = -s * swing_mag
        else:
            l_arm = 0.0; r_arm = 0.0
            
        joints['left_shoulder_pitch_joint'] = l_arm
        joints['right_shoulder_pitch_joint'] = r_arm
        joints['left_elbow_joint'] = 0.3
        joints['right_elbow_joint'] = 0.3
        
        return self.root_pos, joints

    def cycloid_interp(self, start, end, t):
        # Linear X/Y
        res = (1-t)*start + t*end
        
        # Cycloid Z Lift
        # Lift High enough to clear steps (0.15)
        # Add buffer
        clearance = 0.2
        
        # If climbing, we need more lift?
        # Cycloid: Z += R * sin(pi * t)
        
        z_lift = math.sin(t * math.pi) * clearance
        
        res[2] += z_lift
        return res

def main():
    world = World()
    stage = kit.context.get_stage()
    
    # Scene Setup
    create_lighting_array(stage, start_pos=(-2, 0, 0), count=8, spacing=3.0)
    
    # Robot
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    # create_floor_markings includes the concrete floor, removing default ground plane to avoid z-fighting or keeping it lower
    # world.scene.add_default_ground_plane() 
    
    create_floor_markings(world, start_pos=[0,0,0], end_pos=[3,0,0])
    
    # Stairs Params
    S_NUM = 15; S_H = 0.15; S_D = 0.25
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0], num_steps=S_NUM, step_height=S_H, step_depth=S_D)
    
    # Walker
    walker = ProfessionalWalker(start_pos=[0.0, 0.0, 0.72], stair_start=[3.0, 0.0, 0.0], stair_params=(S_D, S_H, S_NUM))
    
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
