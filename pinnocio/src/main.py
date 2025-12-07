import argparse
import time
import os
import rclpy
import sys

from data_loader import HumanPoseSource
from retargeter import G1Retargeter
from robot_publisher import Ros2Publisher

def main():
    parser = argparse.ArgumentParser(description="G1 Retargeting Loop")
    parser.add_argument('--mode', type=str, choices=['udp', 'csv', 'bag'], default='udp', help="Input mode")
    parser.add_argument('--path', type=str, help="Path to input file (csv/bag)")
    args = parser.parse_args()

    # Init ROS2
    rclpy.init()

    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.abspath(os.path.join(script_dir, "../models/g1_description/urdf/g1.urdf"))
    
    # Components
    print(f"[Main] Loading Model from {urdf_path}")
    retargeter = G1Retargeter(urdf_path)
    
    print(f"[Main] Starting Source: {args.mode}")
    source = HumanPoseSource(args.mode, args.path)
    
    print(f"[Main] Starting Publisher")
    publisher = Ros2Publisher(retargeter.model)
    
    print("[Main] Ready. Loop starting...")
    try:
        while rclpy.ok():
            # 1. Get Pose
            pose = source.get_next_pose()
            if pose is None:
                # If file mode, maybe loop or exit?
                if args.mode in ['csv', 'bag']:
                    print("End of file.")
                    break
                time.sleep(0.001)
                continue
            
            # 2. Retarget
            q = retargeter.solve_ik(pose)
            
            # 3. Publish
            publisher.publish(q)
            
            # Spin ROS (for TF broadcasting)
            rclpy.spin_once(publisher, timeout_sec=0.0)
            
            # Limit rate? data_loader handles it for simple file replay
            if args.mode == 'udp':
                time.sleep(0.01) 

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        source.close()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
