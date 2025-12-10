import sys
import os
import math
import time

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

from isaacsim import SimulationApp
CONFIG = {"headless": False, "install_signal_handlers": False, "width": 1280, "height": 720}
kit = SimulationApp(CONFIG)

import omni
import carb
import csv
import numpy as np

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

from pxr import Gf, UsdGeom

# --- CONFIG ---
DEFAULT_CSV_PATH = os.path.join(os.path.dirname(__file__), "recordings/recording_20251206_205703.csv")
ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

KEYPOINT_NAMES = [
    "Nose", "LeftEye", "RightEye", "LeftEar", "RightEar", 
    "LeftShoulder", "RightShoulder", "LeftElbow", "RightElbow", 
    "LeftWrist", "RightWrist", "LeftHip", "RightHip", 
    "LeftKnee", "RightKnee", "LeftAnkle", "RightAnkle"
]

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

class SkeletonVisualizer:
    def __init__(self, keypoint_names):
        self.spheres = []
        self.names = keypoint_names
        
    def setup(self, world):
        self.root_path = "/World/Skeleton_GT"
        for i, name in enumerate(self.names):
            prim_path = f"{self.root_path}/{name}"
            try:
                sphere = VisualSphere(prim_path=prim_path, name=name, position=np.array([0, 0, 0]), scale=np.array([0.05, 0.05, 0.05]), color=np.array([1.0, 0.0, 0.0]))
                self.spheres.append(sphere)
            except: pass

    def update(self, keypoints_3d):
        for i, sphere in enumerate(self.spheres):
            if i < len(keypoints_3d):
                pos = keypoints_3d[i]
                if np.linalg.norm(pos) > 0.01:
                    sphere.set_local_pose(translation=pos)
                    sphere.set_visibility(True)
                else:
                    sphere.set_visibility(False)

class CSVPlayer:
    def __init__(self, file_path, scale=0.8):
        self.file_path = file_path
        self.data = []
        self.headers = []
        self.scale = scale
        self.load()
        
    def load(self):
        if not os.path.exists(self.file_path): return
        with open(self.file_path, 'r') as f:
            reader = csv.reader(f)
            self.headers = next(reader)
            self.col_indices = {}
            for idx, col in enumerate(self.headers):
                if "_X" in col or "_Y" in col or "_Z" in col: self.col_indices[col] = idx
            for row in reader:
                if len(row) > 0: self.data.append(row)
        self.calibrate_height()

    def calibrate_height(self):
        min_z = float('inf')
        for i in range(min(100, len(self.data))):
            indices_y = [self.col_indices.get(f"{n}_Y") for n in ["LeftAnkle", "RightAnkle"] if self.col_indices.get(f"{n}_Y")]
            for idx_y in indices_y:
                 z = -(float(self.data[i][idx_y]) * self.scale)
                 if z < min_z: min_z = z
        self.z_offset = (0.05 - min_z) if min_z != float('inf') else 0.95

    def get_frame_keypoints(self, frame_idx):
        if frame_idx >= len(self.data): return None
        row = self.data[frame_idx]
        points_list = []
        points_dict = {}
        for name in KEYPOINT_NAMES:
            try:
                x_idx = self.col_indices.get(f"{name}_X")
                y_idx = self.col_indices.get(f"{name}_Y")
                z_idx = self.col_indices.get(f"{name}_Z")
                if x_idx and y_idx and z_idx:
                    raw_x = float(row[x_idx]) * self.scale
                    raw_y = float(row[y_idx]) * self.scale
                    raw_z = float(row[z_idx]) * self.scale
                    # Mirroring Fix
                    p = np.array([raw_z, raw_x, -raw_y + getattr(self, 'z_offset', 0.95)])
                    if np.linalg.norm(p) < 0.1 or (raw_x == 0 and raw_y == 0): p = np.array([0,0,0])
                    points_list.append(p)
                    points_dict[name] = p if np.linalg.norm(p) > 0.1 else None
                else: points_list.append(np.array([0,0,0])); points_dict[name] = None
            except: points_list.append(np.array([0,0,0])); points_dict[name] = None
        
        if points_dict.get('LeftHip') is not None and points_dict.get('RightHip') is not None:
             points_dict['Pelvis'] = (points_dict['LeftHip'] + points_dict['RightHip']) / 2.0
             
        return points_list, points_dict, True

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
    
    # Viz
    skeleton_viz = SkeletonVisualizer(KEYPOINT_NAMES)
    skeleton_viz.setup(world)
    player = CSVPlayer(DEFAULT_CSV_PATH, scale=0.75)
    
    # Climber (Start at origin, stairs at X=2.0)
    # Robot is at [0,0,0]. Stairs are at [2,2,0] (World Coords).
    # Wait, create_industrial_stairs uses 'position' as base.
    # If Robot is at 0,0,0, and Walk is along X.
    # We should place stairs at [2, 0, 0] if we want straight walk.
    # User said: "in front of stairs".
    # I will put Stairs at [3, 0, 0] and robot walks X.
    
    # Re-Create Stairs at better pos
    # Actually I can't delete easily, so I'll just rely on the implementation above which I wrote as [2,2,0].
    # So I will make the robot start at [0, 2, 0] or make stairs [3, 0, 0]?
    # Let's fix stairs pos in the function call above.
    
    # Let's re-run scene add with correct pos
    # Actually I can just change the call above.
    # Updated call: position=[3.0, 0.0, 0.0] (Straight ahead on X)
    
    climber = ProceduralClimber(start_pos=[0.5, 0.0, 0.78], stair_start=[3.0, 0.0, 0.0], stair_params=(S_D, S_H, S_NUM))
    
    world.reset()
    
    # Hard Physics
    print("Setting stiff physics...")
    # Attempt to just use Kinematic overrides primarily
    
    frame_idx = 0
    while kit.is_running():
        world.step(render=True)
        
        # 1. Update CSV Viz (Independent)
        if frame_idx < len(player.data):
            res = player.get_frame_keypoints(frame_idx)
            skeleton_viz.update(res[0])
            frame_idx += 1
        else: frame_idx = 0
        
        # 2. Update Procedural Robot
        root_pos, joints = climber.update(0.016) # Assume 60hz dt
        
        g1_robot.set_world_pose(position=root_pos, orientation=np.array([1,0,0,0]))
        g1_robot.set_joint_positions(np.array([joints.get(n, 0.0) for n in g1_robot.dof_names]))
        g1_robot.set_joint_velocities(np.zeros(g1_robot.num_dof))

    kit.close()

if __name__ == "__main__":
    # Monkey patch the function call in main to fix position
    # Actually I wrote main() above, I'll just fix it in the file write
    # I will fix the function call line in the string content.
    sys.modules[__name__].create_industrial_stairs = create_industrial_stairs
    main()
