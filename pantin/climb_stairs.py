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
from pxr import Gf, UsdGeom, UsdLux, Sdf

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
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder


ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

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

def create_lighting_array(stage, start_pos, count=5, spacing=5.0, height=3.0):
    """Adds a row of industrial lights"""
    for i in range(count):
        x = start_pos[0] + i * spacing
        pos = Gf.Vec3f(x, start_pos[1], start_pos[2] + height)
        
        # OpenUSD Light
        light_path = f"/World/Lights/Light_{i}"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateIntensityAttr(30000.0)
        light.CreateRadiusAttr(0.2)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.9, 0.8)) # Warm Industrial
        light.AddTranslateOp().Set(pos)
        
        # Visual Fixture (Sphere)
        # We can use UsdGeom.Sphere or just a VisualSphere wrapper if we had 'world'
        # Since we have stage, let's just make the light visible (SphereLight has geometry in some renderers, but let's be sure)
        pass

# --- KINEMATICS ---

def solve_leg_ik(hip_to_ankle_vec):
    """
    Simple Analytical IK for a 2-segment leg (Thigh + Shin).
    Returns (HipPitch, KneePitch, AnklePitch)
    """
    # G1 Approx Lengths
    L1 = 0.35 # Thigh
    L2 = 0.35 # Shin
    
    dist = np.linalg.norm(hip_to_ankle_vec)
    # Clamp distance
    dist = max(0.1, min(dist, (L1 + L2) * 0.99))
    
    # Law of Cosines
    # c^2 = a^2 + b^2 - 2ab cos(C)
    # Knee Angle (interior)
    # dist^2 = L1^2 + L2^2 - 2*L1*L2*cos(180 - knee_bend)
    # cos(180-knee) = (L1^2 + L2^2 - dist^2) / (2*L1*L2)
    
    try:
        cos_angle = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        interior_knee = math.acos(cos_angle)
        knee_bend = math.pi - interior_knee # How much knee bends from straight
    except:
        knee_bend = 0.0

    # Hip Pitch contribution
    # Angle of vector from Hip to Ankle relative to vertical/forward?
    # In local hip frame:
    # Forward = X, Down = -Z.
    # Pitch = angle in X-Z plane.
    dx = hip_to_ankle_vec[0]
    dz = hip_to_ankle_vec[2]
    
    # Base angle to target
    # atan2(x, -z) -> 0 when straight down (-z), + when forward (x)
    base_pitch = math.atan2(dx, -dz)
    
    # Additional angle due to thigh geometry triangle
    # sin(alpha) / L2 = sin(interior_knee) / dist
    # or Law of Cosines again for hip angle
    # L2^2 = L1^2 + dist^2 - 2*L1*dist*cos(alpha)
    cos_alpha = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    
    # Total Hip Pitch = Base Pitch - Alpha (since knee bends backward usually? No, knee bends forward on humanoids)
    # G1/Human: Knee bends forward (Positive Knee Pitch?).
    # If Knee bends forward, the ankle is 'behind' the line of thigh extension?
    # Let's assume positive knee = bent.
    # Then hip needs to flex (positive pitch) to bring foot forward.
    # Geometry:
    # Hip Angle = base_pitch + alpha?
    # Let's try:
    hip_pitch = base_pitch + alpha
    
    # Ankle Pitch: Keep foot flat (horizontal)
    # Global Foot Angle = HipPitch - KneePitch + AnklePitch = 0
    # AnklePitch = -HipPitch + KneePitch
    ankle_pitch = -hip_pitch + knee_bend
    
    return hip_pitch, knee_bend, ankle_pitch


class KinematicWalker:
    def __init__(self, start_pos, stair_start, stair_params):
        self.root_pos = np.array(start_pos)
        self.stair_start = np.array(stair_start)
        self.stair_params = stair_params # (depth, height, num)
        
        # State
        self.t_cycle = 0.0
        self.cycle_time = 1.0 # Seconds per step
        self.step_length = 0.3
        self.left_swing = True # Left leg swinging first
        
        # Feet Positions (World)
        # Init feet under hips
        self.l_foot = self.root_pos + np.array([0, 0.1, -0.75])
        self.r_foot = self.root_pos + np.array([0, -0.1, -0.75])
        
        # Targets
        self.l_foot_target = self.l_foot.copy()
        self.r_foot_target = self.r_foot.copy()
        self.l_foot_start = self.l_foot.copy()
        self.r_foot_start = self.r_foot.copy()
        
        self.stair_mode = False
        
    def get_terrain_height(self, x):
        # Determine ground height at X
        
        # Flat ground
        if x < self.stair_start[0]:
            return 0.0
            
        # Stairs
        # Relative X
        rx = x - self.stair_start[0]
        # Which step?
        step_idx = int(rx / self.stair_params[0])
        
        if step_idx < 0: return 0.0
        if step_idx >= self.stair_params[2]:
            # Catwalk height
            return self.stair_params[2] * self.stair_params[1]
            
        # On Step
        return (step_idx + 1) * self.stair_params[1]


    def update(self, dt):
        self.t_cycle += dt
        
        # Normalized phase 0..1
        phase = self.t_cycle / self.cycle_time
        
        if phase >= 1.0:
            # Switch Leg
            self.left_swing = not self.left_swing
            self.t_cycle = 0.0
            phase = 0.0
            
            # Lock placed foot, Plan new target for Swing leg
            
            # Move Root Forward logic (Continuous)
            # Actually, we update targets at start of cycle?
            
            # Predict Next Step Position
            # Current Standing Foot is the one that WASN'T swinging (now became stance)
            stance_foot = self.l_foot if not self.left_swing else self.r_foot
            
            # New Target for Swing Foot is StanceX + StepLength
            next_x = stance_foot[0] + self.step_length
            next_z = self.get_terrain_height(next_x)
            
            if self.left_swing:
                self.l_foot_start = self.l_foot.copy()
                self.l_foot_target = np.array([next_x, 0.1, next_z])
            else:
                self.r_foot_start = self.r_foot.copy()
                self.r_foot_target = np.array([next_x, -0.1, next_z])

        # --- UPDATE FEET (Interpolation) ---
        # Swing Leg follows Bezier/Sin curve
        # Stance Leg stays put (Relative to World)
        
        # Swing Height (Lift)
        lift_height = 0.15 
        # Extra lift for stairs?
        if self.l_foot_target[2] > self.l_foot_start[2] + 0.01: # Climbing
            lift_height = 0.25 # Higher lift to clear nose
            
        # Sinusoidal Lift
        z_offset = math.sin(phase * math.pi) * lift_height
        
        # Linear Interp X/Y/Z base
        lerp = phase
        
        swing_pos_base = (1-lerp) * (self.l_foot_start if self.left_swing else self.r_foot_start) + \
                         lerp * (self.l_foot_target if self.left_swing else self.r_foot_target)
                         
        swing_pos = swing_pos_base.copy()
        # Add Z lift
        # Note: We need to lift ABOVE the max of start/end Z to avoid clipping
        base_z = swing_pos[2]
        swing_pos[2] = max(self.l_foot_start[2], self.l_foot_target[2]) + z_offset if self.left_swing else \
                       max(self.r_foot_start[2], self.r_foot_target[2]) + z_offset
        
        if self.left_swing:
            self.l_foot = swing_pos
        else:
            self.r_foot = swing_pos
            
        # --- UPDATE ROOT ---
        # Root should be between feet, smoothed
        # Average X of feet
        avg_x = (self.l_foot[0] + self.r_foot[0]) / 2.0
        avg_z = (self.l_foot[2] + self.r_foot[2]) / 2.0
        
        self.root_pos[0] = avg_x
        # Height: Hip Height above mean foot Z
        self.root_pos[2] = avg_z + 0.75 # Hip Height
        
        # --- SOLVE IK ---
        joints = {}
        
        # Left Leg
        l_vec = self.l_foot - self.root_pos - np.array([0, 0.07, 0]) # Hip Offset
        hp, kp, ap = solve_leg_ik(l_vec)
        joints['left_hip_pitch_joint'] = hp
        joints['left_knee_joint'] = kp
        joints['left_ankle_pitch_joint'] = ap
        
        # Right Leg
        r_vec = self.r_foot - self.root_pos - np.array([0, -0.07, 0])
        hp, kp, ap = solve_leg_ik(r_vec)
        joints['right_hip_pitch_joint'] = hp
        joints['right_knee_joint'] = kp
        joints['right_ankle_pitch_joint'] = ap
        
        # Arms (Simple Sway)
        arm_sway = math.sin(self.t_cycle * math.pi * 2) * 0.5
        joints['left_shoulder_pitch_joint'] = arm_sway
        joints['right_shoulder_pitch_joint'] = -arm_sway
        joints['left_elbow_joint'] = 0.5
        joints['right_elbow_joint'] = 0.5
        
        return self.root_pos, joints

def main():
    world = World()
    
    # Lighting
    stage = kit.context.get_stage()
    create_lighting_array(stage, start_pos=(-2, 0, 0), count=8, spacing=3.0)
    
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    world.scene.add_default_ground_plane()
    
    # Stairs Params
    S_NUM = 15; S_H = 0.15; S_D = 0.25
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0], num_steps=S_NUM, step_height=S_H, step_depth=S_D)
    
    # Walker
    walker = KinematicWalker(start_pos=[0.0, 0.0, 0.78], stair_start=[3.0, 0.0, 0.0], stair_params=(S_D, S_H, S_NUM))
    
    world.reset()
    
    while kit.is_running():
        world.step(render=True)
        
        # Update Walker
        root_pos, joints = walker.update(0.016) 
        
        g1_robot.set_world_pose(position=root_pos, orientation=np.array([1,0,0,0]))
        g1_robot.set_joint_positions(np.array([joints.get(n, 0.0) for n in g1_robot.dof_names]))
        g1_robot.set_joint_velocities(np.zeros(g1_robot.num_dof))

    kit.close()

if __name__ == "__main__":
    main()
