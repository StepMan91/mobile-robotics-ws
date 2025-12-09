import argparse
import os
import sys
import csv
import numpy as np

# 1. Initialize SimulationApp FIRST before importing omni modules
# This is required for standalone Isaac Sim scripts
try:
    from isaacsim import SimulationApp
except ImportError:
    try:
        from omni.isaac.kit import SimulationApp
    except ImportError:
        print("Could not import SimulationApp. Make sure you are running this with the Isaac Sim python environment.")
        sys.exit(1)

CONFIG = {"headless": False, "width": 1280, "height": 720}
kit = SimulationApp(CONFIG)

# 2. Advanced Imports
import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import VisualSphere
from pxr import Gf, UsdGeom

# 3. Configuration
DEFAULT_CSV_PATH = os.path.join(os.path.dirname(__file__), "recordings/recording_20251206_205703.csv")
# Note: Changing backslashes to forward slashes for USD compatibility
ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

# Skeleton Keypoint Mapping (Indices based on typical MediaPipe/Coco topology found in standard CSVs)
# Adjust these indices based on the actual CSV header structure
# Based on previous `head` output:
# Nose, LeftEye, RightEye, LeftEar, RightEar, LeftShoulder, RightShoulder, LeftElbow, RightElbow, LeftWrist, RightWrist...
KEYPOINT_NAMES = [
    "Nose", "LeftEye", "RightEye", "LeftEar", "RightEar", 
    "LeftShoulder", "RightShoulder", "LeftElbow", "RightElbow", 
    "LeftWrist", "RightWrist", "LeftHip", "RightHip", 
    "LeftKnee", "RightKnee", "LeftAnkle", "RightAnkle"
]

class SkeletonVisualizer:
    def __init__(self, keypoint_names):
        self.spheres = []
        self.names = keypoint_names
        
    def setup(self, world):
        # Create a root for the skeleton
        self.root_path = "/World/Skeleton_GT"
        
        for i, name in enumerate(self.names):
            prim_path = f"{self.root_path}/{name}"
            # Create a red sphere for each keypoint
            sphere = VisualSphere(
                prim_path=prim_path,
                name=name,
                position=np.array([0, 0, 0]),
                scale=np.array([0.05, 0.05, 0.05]), # 5cm radius
                color=np.array([1.0, 0.0, 0.0]) # Red
            )
            self.spheres.append(sphere)

    def update(self, keypoints_3d):
        """
        keypoints_3d: List of (x, y, z) tuples/arrays
        """
        for i, sphere in enumerate(self.spheres):
            if i < len(keypoints_3d):
                pos = keypoints_3d[i]
                # Check for valid data (reject 0,0,0 if needed, or confidence check)
                if np.linalg.norm(pos) > 0.01:
                    sphere.set_local_pose(translation=pos)
                    sphere.set_visibility(True)
                else:
                    sphere.set_visibility(False)

class CSVPlayer:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = []
        self.headers = []
        self.load()
        self.current_frame = 0
        
    def load(self):
        print(f"Loading CSV: {self.file_path}")
        with open(self.file_path, 'r') as f:
            reader = csv.reader(f)
            self.headers = next(reader)
            # Find indices for X, Y, Z columns
            self.col_indices = {}
            for idx, col in enumerate(self.headers):
                if "_X" in col or "_Y" in col or "_Z" in col:
                     self.col_indices[col] = idx
            
            for row in reader:
                if len(row) > 0:
                    self.data.append(row)
        print(f"Loaded {len(self.data)} frames.")

    def get_frame_keypoints(self, frame_idx):
        if frame_idx >= len(self.data):
             return []
        
        row = self.data[frame_idx]
        points = []
        
        # We need to extract points in the order of KEYPOINT_NAMES
        # And apply coordinate transform (Camera -> World)
        # RealSense Camera: X-Right, Y-Down, Z-Forward
        # Isaac Sim: X-Forward, Y-Left, Z-Up
        
        # Mapping:
        # Sim X = Cam Z
        # Sim Y = -Cam X
        # Sim Z = -Cam Y (plus offset to lift it up)
        
        for name in KEYPOINT_NAMES:
            try:
                x_idx = self.col_indices.get(f"{name}_X")
                y_idx = self.col_indices.get(f"{name}_Y")
                z_idx = self.col_indices.get(f"{name}_Z")
                
                if x_idx is not None and y_idx is not None and z_idx is not None:
                    raw_x = float(row[x_idx])
                    raw_y = float(row[y_idx])
                    raw_z = float(row[z_idx])
                    
                    # Apply Coordinate Transform
                    sim_x = raw_z
                    sim_y = -raw_x
                    sim_z = -raw_y + 1.2 # approximate height offset if raw is relative to camera
                    
                    points.append(np.array([sim_x, sim_y, sim_z]))
                else:
                     points.append(np.array([0,0,0]))
            except (ValueError, IndexError):
                 points.append(np.array([0,0,0]))
                 
        return points

def main():
    world = World()
    
    # 1. Load Robot
    print(f"Loading Robot from: {ROBOT_USD_PATH}")
    if os.path.exists(ROBOT_USD_PATH):
        add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
        # g1_robot = Robot(prim_path="/World/G1", name="g1")
        # world.scene.add(g1_robot)
    else:
        print(f"[ERROR] Robot USD not found at: {ROBOT_USD_PATH}")
        
    world.scene.add_default_ground_plane()
    
    # 2. Setup Skeleton Visualization
    skeleton_viz = SkeletonVisualizer(KEYPOINT_NAMES)
    skeleton_viz.setup(world)
    
    # 3. Load CSV
    player = CSVPlayer(DEFAULT_CSV_PATH)
    
    world.reset()
    
    print("Starting Simulation Loop...")
    frame_idx = 0
    
    while kit.is_running():
        world.step(render=True)
        
        # Update Skeleton
        if frame_idx < len(player.data):
            points = player.get_frame_keypoints(frame_idx)
            skeleton_viz.update(points)
            frame_idx += 1
        else:
            frame_idx = 0 # Loop
            
    kit.close()

if __name__ == "__main__":
    main()
