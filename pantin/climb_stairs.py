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
from pxr import Gf, UsdGeom

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


class ProceduralClimber:
    def __init__(self, start_pos, stair_start, stair_params):
        self.pos = np.array(start_pos)
        self.stair_start = np.array(stair_start)
        self.stair_params = stair_params # (depth, height, num)
        self.state = "WALK"
        self.t = 0.0
        self.walk_speed = 0.3
        self.climb_speed = 0.2
        self.gait_freq = 3.0
        
    def update(self, dt):
        self.t += dt
        
        # 1. Root Motion
        if self.state == "WALK":
            # Move X towards stair start
            self.pos[0] += self.walk_speed * dt
            if self.pos[0] >= self.stair_start[0]:
                self.state = "CLIMB"
        elif self.state == "CLIMB":
            # Diagonal Motion
            dist = self.pos[0] - self.stair_start[0]
            max_dist = self.stair_params[0] * (self.stair_params[2] + 2) # Steps + Catwalk
            
            if dist < max_dist:
                sx = self.climb_speed * dt
                # Slope
                sz = sx * (self.stair_params[1] / self.stair_params[0])
                self.pos[0] += sx
                if self.pos[2] < (self.stair_params[1] * self.stair_params[2]): # Stop Z at top
                     self.pos[2] += sz
            else:
                self.state = "WAIT"
        
        # 2. Joint Angles (Sine Wave Gait)
        joints = {}
        
        phase = self.t * self.gait_freq
        
        # Legs
        l_hip_pitch = math.sin(phase) * 0.5
        r_hip_pitch = math.sin(phase + math.pi) * 0.5
        
        l_knee = max(0, math.sin(phase + math.pi/2)) * 1.0
        r_knee = max(0, math.sin(phase + math.pi/2 + math.pi)) * 1.0
        
        joints['left_hip_pitch_joint'] = l_hip_pitch
        joints['right_hip_pitch_joint'] = r_hip_pitch
        joints['left_knee_joint'] = l_knee
        joints['right_knee_joint'] = r_knee
        joints['left_ankle_pitch_joint'] = -0.3 # Keep foot somewhat flat
        joints['right_ankle_pitch_joint'] = -0.3

        # Arms (Swing opposite to legs)
        l_shoulder_pitch = -l_hip_pitch * 0.5
        r_shoulder_pitch = -r_hip_pitch * 0.5
        
        joints['left_shoulder_pitch_joint'] = l_shoulder_pitch
        joints['right_shoulder_pitch_joint'] = r_shoulder_pitch
        joints['left_elbow_joint'] = 1.0 # Bent
        joints['right_elbow_joint'] = 1.0
        
        return self.pos, joints

def main():
    world = World()
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    world.scene.add_default_ground_plane()
    
    # Stairs Params
    S_NUM = 15; S_H = 0.15; S_D = 0.25
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0], num_steps=S_NUM, step_height=S_H, step_depth=S_D)
    
    # Climber
    # Start at 0, 0, 0.78 (Hip height)
    climber = ProceduralClimber(start_pos=[0.0, 0.0, 0.78], stair_start=[3.0, 0.0, 0.0], stair_params=(S_D, S_H, S_NUM))
    
    world.reset()
    
    # Hard Physics / Kinematic override
    # We will force positions in the loop
    
    while kit.is_running():
        world.step(render=True)
        
        # Update Procedural Robot
        root_pos, joints = climber.update(0.016) # Assume 60hz dt
        
        g1_robot.set_world_pose(position=root_pos, orientation=np.array([1,0,0,0]))
        g1_robot.set_joint_positions(np.array([joints.get(n, 0.0) for n in g1_robot.dof_names]))
        g1_robot.set_joint_velocities(np.zeros(g1_robot.num_dof))

    kit.close()

if __name__ == "__main__":
    main()
