import sys
import os
import numpy as np

# Mock ROS deps if missing to allow importing modules
try:
    import rclpy
except ImportError:
    print("Mocking rclpy for testing...")
    from unittest.mock import MagicMock
    sys.modules["rclpy"] = MagicMock()
    sys.modules["rclpy.node"] = MagicMock()
    sys.modules["sensor_msgs"] = MagicMock()
    sys.modules["sensor_msgs.msg"] = MagicMock()
    sys.modules["tf2_ros"] = MagicMock()
    sys.modules["geometry_msgs"] = MagicMock()
    sys.modules["geometry_msgs.msg"] = MagicMock()

# Add src to path
sys.path.append(os.path.join(os.getcwd(), 'src'))

try:
    from data_loader import HumanPoseSource
    from retargeter import G1Retargeter
except ImportError as e:
    print(f"Import Error: {e}")
    sys.exit(1)

def test_csv_loading():
    print("Testing CSV Loader...")
    csv_path = r"..\RealSens_body_Tracking_Human Keypoints\recordings\recording_20251206_205703.csv"
    if not os.path.exists(csv_path):
        print(f"CSV not found at {csv_path}")
        return
        
    source = HumanPoseSource('csv', csv_path)
    pose = source.get_next_pose()
    if pose:
        print(f"Success! Loaded pose with {len(pose)} joints.")
        print(f"Sample Joint LeftShoulder: {pose.get('LeftShoulder')}")
    else:
        print("Failed to load any pose from CSV.")

def test_retargeter():
    print("Testing Retargeter (requires Pinocchio)...")
    try:
        urdf_path = os.path.join(os.getcwd(), "models/g1_description/urdf/g1.urdf")
        retargeter = G1Retargeter(urdf_path)
        
        # Test usage
        pose = {
            'Pelvis': np.array([0.0, 0.0, 0.75]),
            'LeftWrist': np.array([0.2, 0.3, 0.4])
        }
        q = retargeter.solve_ik(pose)
        print(f"Solved IK. q shape: {q.shape}")
        print(f"Root Pos: {q[0:3]}")
    except Exception as e:
        print(f"Retargeter Test Failed: {e}")

if __name__ == "__main__":
    test_csv_loading()
    test_retargeter()
