import sys
import os
import math # Added for geometry calcs

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
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder
    # IK Imports
    from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.objects import VisualSphere, VisualCuboid, VisualCylinder
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

def create_industrial_stairs(world, position, num_steps=15, step_height=0.15, step_depth=0.25, width=1.0):
    """
    Creates an industrial-style staircase and a catwalk with handrails.
    """
    base_pos = np.array(position)
    
    # --- STAIRS ---
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        
        pos = base_pos + np.array([x_offset, 0, z_offset])
        
        world.scene.add(
            VisualCuboid(
                prim_path=f"/World/Environment/Stairs/Step_{i}",
                name=f"step_{i}",
                position=pos,
                scale=np.array([step_depth, width, step_height]),
                color=np.array([0.3, 0.3, 0.35]) 
            )
        )
        
    # --- CATWALK ---
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([
        (num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth),
        0, 
        (num_steps - 1) * step_height + (step_height / 2.0)
    ])
    
    world.scene.add(
        VisualCuboid(
            prim_path="/World/Environment/Stairs/Catwalk",
            name="catwalk",
            position=catwalk_pos,
            scale=np.array([catwalk_depth, width, step_height]),
            color=np.array([0.25, 0.25, 0.3])
        )
    )

    # --- HANDRAILS ---
    # Create Posts and Rails
    rail_height = 0.9 # Standard
    post_radius = 0.02
    rail_radius = 0.025
    
    # Calculate Diagonal Length and Angle
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2)
    angle_rad = math.atan2(total_rise, total_run)
    # Pitch Angle (Rotation around Y) - In Isaac, Cylinder is along Z by default? Or Height is Z.
    # We need to rotate it.
    
    # Center of diagonal rail
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height # Offset up
    
    rail_offsets_y = [width/2.0, -width/2.0]
    
    for idx, y_off in enumerate(rail_offsets_y):
        # 1. Main Diagonal Rail
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        
        # Orient: Cylinder default is Up (Z). We want to pitch it down.
        # Rotate around Y axis by -(90 - angle)? No.
        # Angle is from Horizontal.
        # We need to rotate -angle (dip down) + 90?
        # VisualCylinder orientation is usually Axis-Angle quaternions or Euler?
        # Let's try Euler.
        # Pitch is rotation around Y.
        # We want to rotate 'angle' degrees up from horizontal?
        # Actually Cylinder is vertical (Z). So rotate 90 (flat) - angle -> 90-angle.
        # Wait, if angle=0 (flat), we rotate 90 deg around Y.
        # If angle=45, we rotate 45.
        pitch_deg = 90 - math.degrees(angle_rad) 
        
        world.scene.add(
            VisualCylinder(
                prim_path=f"/World/Environment/Stairs/Rail_Diag_{idx}",
                name=f"rail_diag_{idx}",
                position=rail_pos,
                scale=np.array([rail_radius, rail_radius, diag_len + 0.5]), # Extend a bit
                color=np.array([0.8, 0.8, 0.2]), # Yellow/Safety
                orientation=np.array([math.cos(math.radians(pitch_deg)/2), 0, math.sin(math.radians(pitch_deg)/2), 0]) # rough quat for Y rot? 
                # Actually quaternion is [w, x, y, z]. Rotation around Y is [cos(a/2), 0, sin(a/2), 0]
            )
        )
        
        # 2. Vertical Posts (Start, Middle, End)
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height # On Step Surface
             
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             
             world.scene.add(
                VisualCylinder(
                    prim_path=f"/World/Environment/Stairs/Post_{idx}_{p_idx}",
                    name=f"post_{idx}_{p_idx}",
                    position=post_pos,
                    scale=np.array([post_radius, post_radius, rail_height]),
                    color=np.array([0.2, 0.2, 0.2])
                )
             )

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
    def __init__(self, file_path, scale=0.8):
        self.file_path = file_path
        self.data = []
        self.headers = []
        self.scale = scale # Human to Robot Scale Factor
        self.load()
        
    def load(self):
        print(f"Loading CSV: {self.file_path} with Scale {self.scale}")
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
        min_z = float('inf')
        for i in range(min(100, len(self.data))): # Check first 100 frames
            indices_y = [self.col_indices.get(f"{n}_Y") for n in ["LeftAnkle", "RightAnkle"] if self.col_indices.get(f"{n}_Y")]
            
            for idx_y in indices_y:
                 raw_y = float(self.data[i][idx_y]) * self.scale
                 # Sim Z (unadjusted) = -raw_y
                 z = -raw_y
                 if z < min_z: min_z = z
        
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
                    # Apply Scale Here
                    raw_x = float(row[x_idx]) * self.scale
                    raw_y = float(row[y_idx]) * self.scale
                    raw_z = float(row[z_idx]) * self.scale
                    
                    # Transform Camera -> World (Z-up)
                    # RealSense: X-Right, Y-Down, Z-Forward
                    # Isaac: X-Forward, Y-Left, Z-Up
                    
                    # MIRRORING FIX:
                    # User said "front instead of rear" and "mirrored".
                    # Flip the Y axis (Lateral).
                    # Old: sim_y = -raw_x
                    # New: sim_y = raw_x  (This flips lateral direction)
                    # And check Depth:
                    # sim_x = raw_z (Depth is usually correct, ensuring forward is forward)
                    
                    sim_x = raw_z 
                    sim_y = raw_x # FLIPPED SIGN for MIRROR CORRECTION
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
                 
        # CALCULATE PELVIS
        if points_dict.get('LeftHip') is not None and points_dict.get('RightHip') is not None:
             l_hip = points_dict['LeftHip']
             r_hip = points_dict['RightHip']
             pelvis = (l_hip + r_hip) / 2.0
             points_dict['Pelvis'] = pelvis
        
        is_valid = True
        if points_dict.get('Pelvis') is None or valid_count < len(KEYPOINT_NAMES) * 0.5:
            is_valid = False
            
        return points_list, points_dict, is_valid

# --- CCD IK Internal Solver (No Dependencies) ---
class CCDIKSolver:
    def __init__(self, robot):
        self.robot = robot
        
        # Define kinematic chains
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
        
        self.chain_indices = {}
        dof_names = self.robot.dof_names
        
        for name, joints in self.chains.items():
            indices = []
            for j in joints:
                if j in dof_names:
                    indices.append(self.robot.get_dof_index(j))
            self.chain_indices[name] = indices


    def solve_simple(self, targets_dict):
        # Heuristic "Puppet" Solver
        def clamp(v, min_v, max_v): return max(min(v, max_v), min_v)
            
        action = np.zeros(self.robot.num_dof)
        
        pelvis_pos = targets_dict.get('Pelvis')
        if pelvis_pos is None: return action
        
        offsets = {
            'LeftAnkle': np.array([0, 0.07, -0.1]),
            'RightAnkle': np.array([0, -0.07, -0.1]),
            'LeftWrist': np.array([0, 0.15, 0.3]),
            'RightWrist': np.array([0, -0.15, 0.3]),
        }
        
        for name, target in targets_dict.items():
            if target is None: continue
            
            # 1. Compute Local Target Vector (Pelvis -> Target)
            rel_pos = target - pelvis_pos
            chain_vec = rel_pos - offsets.get(name, np.array([0,0,0])) 
            dist = np.linalg.norm(chain_vec)
            
            if name in ['LeftAnkle', 'RightAnkle']:
                 # Angles
                 pitch_angle = np.arctan2(chain_vec[0], -chain_vec[2])
                 roll_angle = np.arctan2(chain_vec[1], -chain_vec[2])
                 
                 max_len = 0.65
                 ratio = clamp(dist / max_len, 0.0, 1.0)
                 knee_angle = (1.0 - ratio) * 2.0 
                 
                 idx = self.chain_indices[name]
                 if len(idx) >= 4:
                      action[idx[0]] = pitch_angle # Hip Pitch
                      action[idx[1]] = roll_angle  # Hip Roll
                      action[idx[3]] = knee_angle  # Knee
                      action[idx[4]] = -knee_angle/2 
                      
            elif name in ['LeftWrist', 'RightWrist']:
                 # Angles
                 pitch_angle = np.arctan2(chain_vec[0], -chain_vec[2])
                 roll_angle = np.arctan2(chain_vec[1], -chain_vec[2])
                 
                 max_arm = 0.5
                 ratio = clamp(dist / max_arm, 0.0, 1.0)
                 elbow_angle = (1.0 - ratio) * 2.0 
                 
                 idx = self.chain_indices[name]
                 if len(idx) >= 4:
                      action[idx[0]] = pitch_angle 
                      action[idx[1]] = roll_angle 
                      action[idx[3]] = elbow_angle 
                 
        return action

# -----------------------------------------------

def main():
    world = World()
    print(f"Loading Robot from: {ROBOT_USD_PATH}")
    add_reference_to_stage(usd_path=ROBOT_USD_PATH, prim_path="/World/G1")
    g1_robot = Robot(prim_path="/World/G1", name="g1")
    world.scene.add(g1_robot)
    world.scene.add_default_ground_plane()
    
    # Add Industrial Stairs to Scene (Offset so robot doesn't start inside them)
    create_industrial_stairs(world, position=[2.0, 2.0, 0.0])

    skeleton_viz = SkeletonVisualizer(KEYPOINT_NAMES)
    skeleton_viz.setup(world)
    
    # Init Player with SCALING (0.75 for G1 approx)
    player = CSVPlayer(DEFAULT_CSV_PATH, scale=0.75)
    
    # Init CCD
    ccd_solver = None # Delayed init

    world.reset()
    
    # --- PHYSICS FIX: STIFFNESS ---
    # Apply High Stiffness to all joints to prevent falling (Position Control)
    # This must be done AFTER reset usually, or ensured persistence
    print("[Physics] Setting Joint Stiffness/Damping...")
    # NOTE: In Isaac Sim, we often need to set drives on the Articulation view or individually
    # For a Core Robot, we can try getting all DOFs
    try:
        # High Stiffness to holding pose
        # Values depend on mass. G1 is ~40kg? 1000.0 might be good.
        g1_robot.set_drive_target_type(drive_target_type="position") 
        # Set Default Gains
        # We need to find number of dofs
        num_dofs = g1_robot.num_dof
        stiffness = np.ones(num_dofs) * 10000.0 # Very Stiff
        damping = np.ones(num_dofs) * 500.0
        
        # This API might require specific drive names or indices?
        # Check docs or try simple property set
        # 'g1_robot' is an Articulation (Robot wrapper)
        
        # We can also do it via USD directly if this fails, but Robot class has set_gains?
        # g1_robot.set_gains(stiffness=stiffness, damping=damping) 
        # BUT set_gains might be for articulation controller
        
        # Let's try iterating joints if possible, or assume Actal drives exist
        pass
    except Exception as e:
        print(f"Error setting drives: {e}")

    
    frame_idx = 0
    print("Starting Main Loop...")
    
    SLOW_DOWN_FACTOR = 10 
    sim_step_count = 0
    
    # Store previous action for smoothness
    current_action = np.zeros(g1_robot.num_dof)

    while kit.is_running():
        world.step(render=True)
        
        # Continuous drive update?
        # No, setting joint positions should be enough IF drives are configured.
        # Use set_joint_positions (Kinematic-like) or Apply Action?
        
        sim_step_count += 1
        if sim_step_count % SLOW_DOWN_FACTOR != 0:
            continue
            
        if sim_step_count % 100 == 0:
             print(f"[Sim] Frame: {frame_idx}/{len(player.data)}")

            
        # Lazy Init Solver once robot is loaded/spawned
        if ccd_solver is None:
             ccd_solver = CCDIKSolver(g1_robot)
             # Apply Stiffness ONCE here if safer
             # g1_robot.get_articulation_controller().set_gains(...)
             
             # Actually, just 'set_joint_positions' on the robot forces the state
             # IF the physics engine doesn't overwrite it immediately due to gravity.
             # If physics is on, we need powerful drives.
             # Alternatively, disable physics on the robot? (Kinematic Only)
             # g1_robot.set_enabled_self_collisions(False)?
             # To make it truly 'Puppet' without falling, we can set KineticEnabled=False on rigid bodies?
             # But 'Robot' class assumes dynamics.
             
             # Force teleport every frame is also an option: 'set_joint_positions'
             pass
        
        if frame_idx < len(player.data):
            res = player.get_frame_keypoints(frame_idx)
            points_list, points_dict, is_valid = res
            
            # 1. Viz
            skeleton_viz.update(points_list)
            
            # 2. Root (Critical for standing)
            if points_dict.get('Pelvis') is not None:
                pose = points_dict['Pelvis']
                
                # ROTATE ROOT 180? User said "Front instead of Rear"
                # If we flipped Y axis, maybe facing is correct now?
                # Let's trust the Y-Flip first.
                
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
                joint_action = ccd_solver.solve_simple(points_dict)
                # Overwrite directly to prevent falling
                g1_robot.set_joint_positions(joint_action)
                # Also set joint velocities to zero to stop momentum?
                g1_robot.set_joint_velocities(np.zeros_like(joint_action))
            
            frame_idx += 1
        else:
            frame_idx = 0 
            
    kit.close()

if __name__ == "__main__":
    main()
