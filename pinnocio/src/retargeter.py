import pinocchio as pin
import numpy as np
import os
import math

class G1Retargeter:
    def __init__(self, urdf_path):
        # Load Model
        # Pinocchio requires package dirs to find meshes
        package_dir = os.path.abspath(os.path.join(os.path.dirname(urdf_path), "../../.."))
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        # Geometry not strictly needed for IK unless collision avoidance used
        # self.geom_model = pin.buildGeomModelFromUrdf(self.model, urdf_path, pin.GeometryType.VISUAL, package_dirs=[package_dir])
        # self.geom_data = pin.GeometryObjectVector()
        
        self.q0 = pin.neutral(self.model)
        
        # Mapping: Human Keypoint Name -> G1 Frame Name
        # Note: G1 Frames are Link names or Joint frames
        self.keypoint_map = {
            'LeftWrist': 'left_wrist_roll_link',
            'RightWrist': 'right_wrist_roll_link',
            'LeftAnkle': 'left_ankle_roll_link',
            'RightAnkle': 'right_ankle_roll_link',
            # 'Pelvis': 'pelvis', # Root handled separately
        }
        
        self.frame_ids = {}
        for k, v in self.keypoint_map.items():
            if self.model.existFrame(v):
                self.frame_ids[k] = self.model.getFrameId(v)
            else:
                print(f"[Retargeter] Warning: Frame {v} not found in model")

        # Config
        self.dt = 0.01
        self.damp = 1e-4
        self.q = self.q0.copy()

    def solve_ik(self, human_pose):
        """
        human_pose: Dict[str, np.array(3)] (Positions in ROS frame)
        Returns: q (numpy array of joint angles)
        """
        if not human_pose:
            return self.q0

        # 1. Root Pose
        # Human Root (Pelvis) -> Robot Root (pelvis)
        # We need to offset the human root to match robot height/scale
        # For now, simple direct mapping of Delta or Absolute if calibrated
        
        h_root = human_pose.get('Pelvis')
        if h_root is not None:
            # Assume robot starts at (0,0,0.75) roughly
            # If human root is (x,y,z), we might want to preserve z height relative to floor
            # But human data might be camera-relative.
            # Let's assume input is already decent or filtered.
            
            # Simple Pass-through for Root Position (Free Flyer)
            # The first 7 vars of q are [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
            self.q[0] = h_root[0]
            self.q[1] = h_root[1]
            self.q[2] = h_root[2] # Ensure this >= 0.6 for G1 stability
        
        # 2. Iterate IK (CLIK)
        # J * dq = dx
        # minimize | J*dq - dx |^2 + lambda * |dq|^2
        
        for _ in range(5): # Iterations
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            
            J_list = []
            err_list = []
            
            # Postural Task (keep close to neutral)
            # J_posture = Identity
            # err_posture = k * (q0 - q)
            # We add this as regularization or secondary task.
            # Here simplified: Damped LS handles small movements.
            
            for h_name, f_id in self.frame_ids.items():
                if h_name in human_pose:
                    target_pos = human_pose[h_name]
                    
                    # Current Frame Pos
                    curr_tf = self.data.oMf[f_id]
                    curr_pos = curr_tf.translation
                    
                    err = target_pos - curr_pos
                    
                    # Get Frame Jacobian (6D) - take top 3 rows for Position
                    J = pin.computeFrameJacobian(self.model, self.data, self.q, f_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                    J_pos = J[:3, :]
                    
                    J_list.append(J_pos)
                    err_list.append(err)
            
            if not J_list:
                break
                
            J_stack = np.vstack(J_list)
            err_stack = np.hstack(err_list)
            
            # Solve Damped Least Squares
            # dq = J.T * inv(J*J.T + damp*I) * err
            # Or using numpy lstsq
            
            dq = np.linalg.lstsq(J_stack, err_stack, rcond=None)[0]
            
            # Integrate
            self.q = pin.integrate(self.model, self.q, dq * 0.5) # Gain 0.5 for stability
            
        return self.q
