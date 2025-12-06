#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import socket
import json
import threading
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

class HumanBridgeNode(Node):
    def __init__(self):
        super().__init__('human_bridge_node')
        self.publisher_ = self.create_publisher(MarkerArray, 'human_skeleton', 10)
        self.js_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.get_logger().info('Initialized HumanBridgeNode with JointState publisher')
        self.declare_parameter('udp_port', 8888)
        self.port = self.get_parameter('udp_port').value
        
        self.get_logger().info(f'Starting UDP Bridge on port {self.port}...')
        
        # Start UDP listener in a separate thread
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(1.0) # Timeout to allow checking for shutdown
        self.running = True
        self.thread = threading.Thread(target=self.udp_listener)
        self.thread.daemon = True
        self.thread.start()

    def udp_listener(self):
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)
                if not hasattr(self, '_last_log') or (time.time() - self._last_log > 1.0):
                    self.get_logger().info(f'Received UDP packet from {addr}, len={len(data)}')
                    self._last_log = time.time()
                json_str = data.decode('utf-8')
                self.process_data(json_str)
            except socket.timeout:
                continue
            except OSError:
                if not self.running:
                    break
                self.get_logger().error(f'UDP Error: OSError')
            except Exception as e:
                self.get_logger().error(f'UDP Error: {e}')

    def process_data(self, json_str):
        try:
            data = json.loads(json_str)
            skeletons = data.get('skeletons', [])
            
            marker_array = MarkerArray()
            
            for skel in skeletons:
                skel_id = skel['id']
                joints = skel['joints']
                
                # Create a sphere marker for each joint
                for joint_name, joint_data in joints.items():
                    marker = Marker()
                    marker.header.frame_id = "camera_link" 
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"person_{skel_id}"
                    
                    # Map joint name to ID for stable tracking
                    # Simple has using sum of chars or similar to avoid collision
                    marker.id = (skel_id * 1000) + (hash(joint_name) % 1000)
                    
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # Coordinate transform (Camera -> ROS)
                    # Camera: X=Right, Y=Down, Z=Forward
                    # ROS: X=Forward, Y=Left, Z=Up
                    # Mapping: ROS_X = Cam_Z, ROS_Y = -Cam_X, ROS_Z = -Cam_Y
                    
                    marker.pose.position.x = float(joint_data['z']) 
                    marker.pose.position.y = -float(joint_data['x']) 
                    marker.pose.position.z = -float(joint_data['y']) 
                    
                    marker.pose.orientation.w = 1.0
                    
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    
                    marker.color.a = 1.0
                    marker.color.r = 1.0 if "Wrist" in joint_name else 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    
                    marker_array.markers.append(marker)
            
            self.publisher_.publish(marker_array)
            
            # Retarget first skeleton to Robot
            if skeletons:
                self.solve_ik_and_publish(skeletons[0])

        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')

    def solve_ik_and_publish(self, skel):
        joints = skel.get('joints', {})
        
        # Helper to get numpy point from UDP data
        def get_pos(name):
            if name not in joints: return None
            # Transform to ROS frame (matches visualize markers)
            # ROS_X = Cam_Z, ROS_Y = -Cam_X, ROS_Z = -Cam_Y
            d = joints[name]
            return np.array([float(d['z']), -float(d['x']), -float(d['y'])])

        # --- 1. Root Pose (Pelvis) ---
        # Try to find a root. MidHip or average of Hips.
        root_pos = get_pos('Pelvis') or get_pos('Waist')
        if root_pos is None:
            l_hip = get_pos('LeftHip')
            r_hip = get_pos('RightHip')
            if l_hip is not None and r_hip is not None:
                root_pos = (l_hip + r_hip) / 2.0
        
        if root_pos is not None:
            # Publish TF map -> pelvis
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'pelvis'
            t.transform.translation.x = root_pos[0]
            t.transform.translation.y = root_pos[1]
            t.transform.translation.z = root_pos[2]
            # Fixed orientation for now (standing up)
            # Todo: Calculate rotation from hips/shoulders
            t.transform.rotation.w = 1.0 
            self.tf_broadcaster_.sendTransform(t)

        # --- 2. Joint States ---
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define joints to zero out if missing
        cmd_joints = {}

        # --- Arms Logic ---
        # Simple vector based angle calculation
        def get_vector(n1, n2):
            p1, p2 = get_pos(n1), get_pos(n2)
            if p1 is None or p2 is None: return None
            v = p2 - p1
            return v / np.linalg.norm(v)

        def angle_between(v1, v2):
            if v1 is None or v2 is None: return 0.0
            dot = np.dot(v1, v2)
            return math.acos(np.clip(dot, -1.0, 1.0))

        # Left Arm
        # Shoulder Pitch: Angle between Arm Vector and Vertical Z
        # Shoulder Roll: Angle between Arm Vector and Side Plane
        
        # Heuristic: 
        # Elbow: Angle between Upper and Lower arm.
        # Shoulder: Simple spherical mapping
        
        # Left Side
        l_shoulder = get_pos('LeftShoulder')
        l_elbow = get_pos('LeftElbow')
        l_wrist = get_pos('LeftWrist')
        
        if l_shoulder is not None and l_elbow is not None:
            # Upper arm vector
            u_arm = (l_elbow - l_shoulder)
            u_arm_norm = u_arm / np.linalg.norm(u_arm)
            
            # G1 Left Shoulder Pitch (Rotate around Y) - Forward/Back
            # Project to XZ plane
            # Angle from -Z (down) to projected vector
            # But G1 zero is arm straight down? Let's assume standard T-pose or I-pose.
            # Usually I-pose (arms down).
            
            # Pitch: Angle projected on XZ plane. At rest (down), vector is (0,0,-1).
            pitch = math.atan2(u_arm_norm[0], -u_arm_norm[2]) 
            cmd_joints['left_shoulder_pitch_joint'] = pitch
            
            # Roll: Angle projected on YZ plane. 
            # Outward is +Y. 
            roll = math.atan2(u_arm_norm[1], -u_arm_norm[2])
            cmd_joints['left_shoulder_roll_joint'] = roll

            if l_wrist is not None:
                # Elbow Flexion
                l_arm = (l_wrist - l_elbow)
                l_arm_norm = l_arm / np.linalg.norm(l_arm)
                
                # Angle between upper and lower
                # G1 elbow is 0 at straight?
                flexion = angle_between(-u_arm_norm, l_arm_norm) 
                
                # Better Elbow:
                # v1 = Shoulder->Elbow
                # v2 = Elbow->Wrist
                # Angle difference.
                elbow_angle = angle_between(u_arm_norm, l_arm_norm)
                cmd_joints['left_elbow_joint'] = elbow_angle

        # Right Side (Mirror logic)
        r_shoulder = get_pos('RightShoulder')
        r_elbow = get_pos('RightElbow')
        r_wrist = get_pos('RightWrist')
        
        if r_shoulder is not None and r_elbow is not None:
            u_arm = (r_elbow - r_shoulder)
            u_arm_norm = u_arm / np.linalg.norm(u_arm)
            
            pitch = math.atan2(u_arm_norm[0], -u_arm_norm[2])
            cmd_joints['right_shoulder_pitch_joint'] = pitch
            
            # Roll: For right arm, Y is negative outward?
            # ROS frame: Y is Left. So Right is -Y.
            # If arm is at -Y, atan2(-1, 0) = -PI/2.
            # We want positive roll for abduction? URDF limit is -2.9 to 0.5.
            roll = math.atan2(u_arm_norm[1], -u_arm_norm[2])
            cmd_joints['right_shoulder_roll_joint'] = roll

            if r_wrist is not None:
                l_arm = (r_wrist - r_elbow)
                l_arm_norm = l_arm / np.linalg.norm(l_arm)
                elbow_angle = angle_between(u_arm_norm, l_arm_norm)
                cmd_joints['right_elbow_joint'] = elbow_angle

        # Legs (Hip Pitch, Knee) - Simplified
        # ... Similar logic or just keep zero for now if legs not critical ...
        # Adding Basic Hip/Knee
        
        for side, prefix in [('Left', 'left'), ('Right', 'right')]:
            hip = get_pos(f'{side}Hip')
            knee = get_pos(f'{side}Knee')
            ankle = get_pos(f'{side}Ankle')
            
            if hip is not None and knee is not None:
                u_leg = (knee - hip)
                u_leg_norm = u_leg / np.linalg.norm(u_leg)
                
                # Hip Pitch (X-Z plane)
                # Down is (0,0,-1). Forward is X.
                pitch = math.atan2(u_leg_norm[0], -u_leg_norm[2])
                cmd_joints[f'{prefix}_hip_pitch_joint'] = pitch
                
                # Hip Roll (Y-Z plane)
                roll = math.atan2(u_leg_norm[1], -u_leg_norm[2])
                cmd_joints[f'{prefix}_hip_roll_joint'] = roll

                if ankle is not None:
                    l_leg = (ankle - knee)
                    l_leg_norm = l_leg / np.linalg.norm(l_leg)
                    knee_angle = angle_between(u_leg_norm, l_leg_norm)
                    cmd_joints[f'{prefix}_knee_joint'] = knee_angle

        # Populate message
        js_msg.name = list(cmd_joints.keys())
        js_msg.position = list(cmd_joints.values())
        
        if js_msg.name:
            self.js_publisher_.publish(js_msg)

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HumanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
