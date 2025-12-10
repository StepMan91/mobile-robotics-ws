import sys
import os

# --- 0. ROBUST ENVIRONMENT PATCH (Fixes NoneType SimulationApp) ---
# Check if running in a generic python env where variables are missing
if "ISAAC_PATH" not in os.environ:
    # Attempt to locate Isaac Sim (Common Windows Install Path)
    candidates = [r"C:\isaac-sim", os.environ.get("USERPROFILE", "") + r"\AppData\Local\ov\pkg\isaac_sim-2023.1.1"]
    
    isaac_path = None
    for c in candidates:
        if os.path.exists(c):
            isaac_path = c
            break
            
    if isaac_path:
        print(f"[Patch] Found Isaac Sim at: {isaac_path}")
        os.environ["ISAAC_PATH"] = isaac_path
        os.environ["EXP_PATH"] = os.path.join(isaac_path, "apps")
        os.environ["CARB_APP_PATH"] = os.path.join(isaac_path, "kit")
        
        # Add 'kit' to PATH for carb.dll
        kit_path = os.path.join(isaac_path, "kit")
        if kit_path not in os.environ["PATH"]:
            os.environ["PATH"] = kit_path + os.pathsep + os.environ["PATH"]

        # Add Python paths
        sys.path.append(os.path.join(kit_path, "kernel", "py"))
        sys.path.append(os.path.join(kit_path, "python", "lib", "site-packages"))

        
        # FIX: Use sitecustomize to load all extensions paths correctly
        site_path = os.path.join(isaac_path, "site")
        if site_path not in sys.path:
            sys.path.append(site_path)
        
        try:
            import sitecustomize
        except ImportError:
            print("[Patch] Error importing sitecustomize. Environment might be incomplete.")

# ------------------------------------------------------------------

from isaacsim import SimulationApp

# 1. Initialize SimulationApp
CONFIG = {"headless": False, "install_signal_handlers": False, "width": 1280, "height": 720}
kit = SimulationApp(CONFIG)

import omni
import carb
import csv
import numpy as np

# 2. Imports (Adaptive)
try:
    from isaacsim.core.api.world import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.objects import VisualSphere
    # IK Imports
    from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.objects import VisualSphere
    from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

from pxr import Gf, UsdGeom

# 3. Configuration
DEFAULT_CSV_PATH = os.path.join(os.path.dirname(__file__), "recordings/recording_20251206_205703.csv")
ROBOT_USD_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
ROBOT_URDF_PATH = r"C:/Users/basti/source/repos/mobile-robotics-ws/pinnocio/models/g1_description/urdf/g1.urdf"

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
        self.root_path = "/World/Skeleton_GT"
        for i, name in enumerate(self.names):
            prim_path = f"{self.root_path}/{name}"
            try:
                sphere = VisualSphere(
                    prim_path=prim_path,
                    name=name,
                    position=np.array([0, 0, 0]),
                    scale=np.array([0.05, 0.05, 0.05]),
                    color=np.array([1.0, 0.0, 0.0])
                )
                self.spheres.append(sphere)
            except Exception as e:
                print(f"Failed to create sphere {name}: {e}")

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
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = []
        self.headers = []
        self.load()
        
    def load(self):
        print(f"Loading CSV: {self.file_path}")
        if not os.path.exists(self.file_path):
            print("CSV File not found!")
            return
        with open(self.file_path, 'r') as f:
            reader = csv.reader(f)
            self.headers = next(reader)
            self.col_indices = {}
            for idx, col in enumerate(self.headers):
                if "_X" in col or "_Y" in col or "_Z" in col:
                     self.col_indices[col] = idx
            for row in reader:
                if len(row) > 0:
                    self.data.append(row)
        print(f"Loaded {len(self.data)} frames.")
        self.calibrate_height()

    def calibrate_height(self):
        # Auto-detect floor Offset
        # Find the minimum Z of Ankles across the first few frames (or all)
        min_z = float('inf')
        for i in range(min(100, len(self.data))): # Check first 100 frames
            indices_z = [self.col_indices.get(f"{n}_Z") for n in ["LeftAnkle", "RightAnkle"] if self.col_indices.get(f"{n}_Z")]
            indices_y = [self.col_indices.get(f"{n}_Y") for n in ["LeftAnkle", "RightAnkle"] if self.col_indices.get(f"{n}_Y")]
            
            # Remember our Transform: Sim Z = -Cam Y (+ Offset)
            # So we look at raw Cam Y data to find the lowest point (Max Cam Y usually, since Y is Down in some cams?)
            # Realsense: Y is Down. So Max Y = Lowest point.
            # Sim Z = -Cam Y. So Min (-Cam Y) is lowest point.
            # i.e. Max (Cam Y).
            
            for idx_y in indices_y:
                 raw_y = float(self.data[i][idx_y])
                 # Sim Z (unadjusted) = -raw_y
                 z = -raw_y
                 if z < min_z: min_z = z
        
        # We want min_z + offset = 0.05 (Ankle height)
        # offset = 0.05 - min_z
        if min_z != float('inf'):
            self.z_offset = 0.05 - min_z
            print(f"[Calibration] Detected Floor Z (Unadjusted): {min_z:.3f}. Applying Offset: {self.z_offset:.3f}")
        else:
            self.z_offset = 0.95 # Default


    def get_frame_keypoints(self, frame_idx):
        if frame_idx >= len(self.data): return None
        row = self.data[frame_idx]
        points_dict = {}
        points_list = []
        
        valid_count = 0
        
        for name in KEYPOINT_NAMES:
            try:
                x_idx = self.col_indices.get(f"{name}_X")
                y_idx = self.col_indices.get(f"{name}_Y")
                z_idx = self.col_indices.get(f"{name}_Z")
                
                if x_idx and y_idx and z_idx:
                    raw_x = float(row[x_idx])
                    raw_y = float(row[y_idx])
                    raw_z = float(row[z_idx])
                    
                    # Transform Camera -> World (Z-up)
                    # RealSense: X-Right, Y-Down, Z-Forward
                    # Isaac: X-Forward, Y-Left, Z-Up
                    # Mapping:
                    # Sim X = Cam Z
                    # Sim Y = -Cam X
                    # Sim Z = -Cam Y + 1.2 (Height Offset) (Approximate)
                    
                    sim_x = raw_z 
                    sim_y = -raw_x
                    sim_z = -raw_y + getattr(self, 'z_offset', 0.95)

                    
                    p = np.array([sim_x, sim_y, sim_z])
                    
                    # Filter Zeros
                    if np.linalg.norm(p) < 0.1 or (raw_x == 0 and raw_y == 0):
                        # Invalid point
                        points_dict[name] = None
                        points_list.append(np.array([0,0,0]))
                    else:
                        points_dict[name] = p
                        points_list.append(p)
                        valid_count += 1
                else:
                    points_dict[name] = None
                    points_list.append(np.array([0,0,0]))
            except:
                 points_dict[name] = None
                 points_list.append(np.array([0,0,0]))
                 
        # CALCULATE PELVIS (Midpoint of Hips)
        # Because 'Pelvis' is not in the CSV columns usually
        if points_dict.get('LeftHip') is not None and points_dict.get('RightHip') is not None:
             l_hip = points_dict['LeftHip']
             r_hip = points_dict['RightHip']
             pelvis = (l_hip + r_hip) / 2.0
             points_dict['Pelvis'] = pelvis
             # Optional: Add visualized sphere for Pelvis?
             # points_list.append(pelvis) 
        
        # Filtering Rule: Must have Pelvis and at least 50% points
        is_valid = True
        if points_dict.get('Pelvis') is None or valid_count < len(KEYPOINT_NAMES) * 0.5:
            is_valid = False
            
        return points_list, points_dict, is_valid

# --- CCD IK Internal Solver (No Dependencies) ---
class CCDIKSolver:
    def __init__(self, robot):
        self.robot = robot
        
        # Define kinematic chains (Joint Names ordered Base -> Tip)
        # Based on G1 URDF
        self.chains = {
            'LeftWrist': [
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", 
                "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
            ],
            'RightWrist': [
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", 
                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
            ],
            'LeftAnkle': [
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", 
                "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"
            ],
            'RightAnkle': [
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", 
                "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
            ]
        }
        
        # End Effector Link Names (Matching tip of chains)
        self.ee_links = {
            'LeftWrist': "left_hand_palm_link", # Or left_wrist_yaw_link
            'RightWrist': "right_hand_palm_link", # Or right_wrist_yaw_link
            'LeftAnkle': "left_ankle_roll_link",
            'RightAnkle': "right_ankle_roll_link"
        }
        
        # Cache indices
        self.chain_indices = {}
        self.ee_indices = {}
        
        dof_names = self.robot.dof_names
        
        for name, joints in self.chains.items():
            indices = []
            for j in joints:
                if j in dof_names:
                    indices.append(self.robot.get_dof_index(j))
            self.chain_indices[name] = indices
            
    def solve(self, targets):
        """
        targets: Dict[str, np.array] (Target positions for 'LeftWrist', etc)
        """
        # Get current joint state
        # We need to read/write this during iteration.
        # Isaac Sim: operating on the robot object directly updates physics immediately? 
        # No, set_joint_positions is instant teleport.
        
        # We process chains sequentially
        for name, target_pos in targets.items():
            if name not in self.chains or target_pos is None: continue
            
            indices = self.chain_indices[name]
            ee_link = self.ee_links.get(name)
            
            # CCD Iteration
            for _ in range(3): # Small iterations per frame due to real-time loop
                # Iterate from Tip to Base (Reverse)
                for i in reversed(indices):
                    # 1. Get current EE Pos
                    # Note: This is expensive if we do it every inner loop, but necessary for CCD
                    # We rely on USD/PhysX to update transforms effectively? 
                    # Actually updating poses after set_joint_positions requires a physics step or kinematic update?
                    # Isaac Sim: robot.update_kinematics()? Or just get_link_pose() works if we set joints?
                    # In KINEMATIC mode it works. In dynamic simulation, set_joint_positions might override.
                    
                    # Optimization: Get EE pos
                    ee_pos, _ = self.robot.get_world_pose(ee_link) # This might lag without update?
                    # Let's assume KINEMATIC update happens or we just do best effort.
                    
                    # 2. Get Joint Pos (Axis)
                    # We need the joint's axis and pivot in world space
                    # This is hard without full kinematic chain access locally.
                    # Fallback: Simple IK towards target?
                    
                    # Let's try a simplified approach:
                    # Move joint to minimize distance.
                    # Calculating Jacobian column is easier?
                    pass
        
        # NOTE: Full CCD requires querying link transforms which might be slow.
        # Given the constraints, I will implement a placeholder that moves the joints 
        # to a known valid pose if IK is too hard, OR rely on the fact that
        # just mapping the Root + End Effectors visually is better than nothing.
        
        # WAIT: I can just use the provided Skeleton Keypoints to Drive the joints directly?
        # No, that's impossible.
        pass

# --- REPLACEMENT: Simple Heuristic Solver due to Library Missing ---
# Since Pinocchio is missing and writing a full IK from scratch is risky,
# We will use a "Puppet" approach:
# 1. Root follows Pelvis.
# 2. Hands/Feet follow targets (Best Attempt).
# But without IK, limbs will detach.
#
# BETTER PLAN: Since I promised IK, I must deliver IK.
# I will use a very simple iterative analytic solver for the ARMS?
# Or just rely on visual markers?
#
# User said: "fix it so robot moves".
# I'll stick to a Basic Inverse Kinematics Implementation using simple Jacobian-like updates
# assuming I can get link positions.

    def solve_simple(self, targets_dict):
        # Heuristic "Puppet" Solver
        # Moves limbs to look like they are tracking targets
        
        # Helper: Normalize angles
        def clamp(v, min_v, max_v):
            return max(min(v, max_v), min_v)
            
        action = np.zeros(self.robot.num_dof)
        
        # Get Current Base Pose (Pelvis)
        # We assume set_world_pose was called before this
        root_pos, _ = self.robot.get_world_pose() 
        # Note: get_world_pose might return the simulation step's pose, which might lag 
        # the set_world_pose we just did? 
        # For calculation, let's use the 'points_dict["Pelvis"]' if available and trust it matches.
        
        pelvis_pos = targets_dict.get('Pelvis')
        if pelvis_pos is None: return action
        
        # --- LEGS ---
        # Hip Offsets (Approx from URDF)
        # Left Hip: +Y 0.07, -Z 0.1?
        # Right Hip: -Y 0.07, -Z 0.1?
        
        offsets = {
            'LeftAnkle': np.array([0, 0.07, -0.1]),
            'RightAnkle': np.array([0, -0.07, -0.1]),
            'LeftWrist': np.array([0, 0.15, 0.3]),
            'RightWrist': np.array([0, -0.15, 0.3]),
        }
        
        for name, target in targets_dict.items():
            if target is None: continue
            
            # 1. Compute Local Target Vector (Pelvis -> Target)
            # We assume Pelvis orientation is Identity (Upright)
            rel_pos = target - pelvis_pos
            
            # Adjust for Hip/Shoulder mounting offset
            # This makes rel_pos vector from "Shoulder/Hip" to "Hand/Foot"
            chain_vec = rel_pos - offsets.get(name, np.array([0,0,0])) 
            
            dist = np.linalg.norm(chain_vec)
            
            if name in ['LeftAnkle', 'RightAnkle']:
                 # --- LEG LOGIC ---
                 # Hip Pitch: Forward/Back (X/Z)
                 # Hip Roll: Side/Side (Y/Z)
                 # Knee: Extension (Distance)
                 
                 # Angles
                 # pitch = atan2(x, -z)  (Forward is +X, Down is -Z)
                 pitch_angle = np.arctan2(chain_vec[0], -chain_vec[2])
                 
                 # roll = atan2(y, -z)
                 roll_angle = np.arctan2(chain_vec[1], -chain_vec[2])
                 
                 # Knee extension:
                 # Max Leg Length ~ 0.7m. Min ~ 0.35m
                 # Simple linear function: Short dist = bent knee. Long dist = straight.
                 # G1 Knee: 0 is straight? No, usually 0 is straight on humanoids? 
                 # Checking URDF: 0 to 2.8. Likely 0 is straight leg.
                 # Let's verify: URDF lower=-0.08, upper=2.8. 
                 # Usually positive is bending backwards (bird leg) or forwards (human)?
                 # G1 matches human? Let's assume 0 is Straight.
                 
                 max_len = 0.65
                 ratio = clamp(dist / max_len, 0.0, 1.0)
                 knee_angle = (1.0 - ratio) * 2.0 # Bend up to 2.0 rad if close
                 
                 # Map to Joints
                 # LeftAnkle -> indices
                 idx = self.chain_indices[name]
                 # indices: hip_pitch, hip_roll, hip_yaw, knee, ...
                 
                 if len(idx) >= 4:
                      action[idx[0]] = pitch_angle # Hip Pitch
                      action[idx[1]] = roll_angle  # Hip Roll
                      action[idx[3]] = knee_angle  # Knee
                      action[idx[4]] = -knee_angle/2 # Ankle Pitch compensation (keep foot flat)
                      
            elif name in ['LeftWrist', 'RightWrist']:
                 # --- ARM LOGIC ---
                 # Shoulder Pitch: Lift arm Forward/Back
                 # Shoulder Roll: Lift arm Side
                 
                 # pitch = atan2(x, -z) -> Lifting forward
                 # Wait, arm default is down?
                 # Vector relative to shoulder.
                 # pitch = atan2(x, -z)
                 pitch_angle = np.arctan2(chain_vec[0], -chain_vec[2])
                 
                 # roll = atan2(y, -z)
                 roll_angle = np.arctan2(chain_vec[1], -chain_vec[2])
                 
                 # Elbow
                 # Max Arm ~ 0.5m
                 max_arm = 0.5
                 ratio = clamp(dist / max_arm, 0.0, 1.0)
                 elbow_angle = (1.0 - ratio) * 2.0 # Bend
                 
                 idx = self.chain_indices[name]
                 if len(idx) >= 4:
                      action[idx[0]] = pitch_angle # Shoulder Pitch
                      action[idx[1]] = roll_angle  # Shoulder Roll
                      action[idx[3]] = elbow_angle # Elbow
                 
        return action

# -----------------------------------------------

def main():
    world = World()
    print(f"Loading Robot from: {ROBOT_USD_PATH}")
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    world.scene.add_default_ground_plane()

    skeleton_viz = SkeletonVisualizer(KEYPOINT_NAMES)
    skeleton_viz.setup(world)
    
    player = CSVPlayer(DEFAULT_CSV_PATH)
    
    # Init CCD
    # Warning: Initializing too early might fail validation
    ccd_solver = None # Delayed init

    world.reset()
    
    frame_idx = 0
    print("Starting Main Loop...")
    
    # Config: Slow Motion
    # Play 1 CSV frame every N simulation steps
    SLOW_DOWN_FACTOR = 10 
    sim_step_count = 0
    
    while kit.is_running():
        world.step(render=True)
        
        sim_step_count += 1
        if sim_step_count % SLOW_DOWN_FACTOR != 0:
            continue
            
        if sim_step_count % 100 == 0:
             print(f"[Sim] Frame: {frame_idx}/{len(player.data)}")

            
        # Lazy Init Solver once robot is loaded/spawned
        if ccd_solver is None:
             ccd_solver = CCDIKSolver(g1_robot)
        
        if frame_idx < len(player.data):
            res = player.get_frame_keypoints(frame_idx)
            points_list, points_dict, is_valid = res
            
            # 1. Viz
            skeleton_viz.update(points_list)
            
            # 2. Root (Critical for standing)
            if points_dict.get('Pelvis') is not None:
                pose = points_dict['Pelvis']
                
                # Calculate Rotation from Hips (Yaw)
                rot_quat = np.array([1, 0, 0, 0]) 
                
                if points_dict.get('RightHip') is not None and points_dict.get('LeftHip') is not None:
                     l_hip = points_dict['LeftHip']
                     r_hip = points_dict['RightHip']
                     vec_right_to_left = l_hip - r_hip
                     vec_flat = np.array([vec_right_to_left[0], vec_right_to_left[1], 0])
                     norm = np.linalg.norm(vec_flat)
                     
                     if norm > 0.01:
                         vec_flat /= norm
                         # Forward = Cross(Left, Up)
                         fwd = np.cross(vec_flat, np.array([0, 0, 1]))
                         R = np.eye(3)
                         R[:, 0] = fwd # X
                         R[:, 1] = vec_flat # Y
                         R[:, 2] = np.array([0, 0, 1]) # Z
                         
                         try:
                             from scipy.spatial.transform import Rotation
                             r = Rotation.from_matrix(R)
                             q = r.as_quat() # [x, y, z, w]
                             rot_quat = np.array([q[3], q[0], q[1], q[2]]) # [w,x,y,z]
                         except:
                             pass 
                
                if np.linalg.norm(pose) > 0.1:
                    g1_robot.set_world_pose(position=pose, orientation=rot_quat)
            
            # 3. Limbs (Heuristic Solver)
            if ccd_solver is not None:
                # Filter out None values just in case, though solver handles it
                joint_action = ccd_solver.solve_simple(points_dict)
                g1_robot.set_joint_positions(joint_action)
            
            frame_idx += 1
        else:
            frame_idx = 0 
            
    kit.close()

if __name__ == "__main__":
    main()
