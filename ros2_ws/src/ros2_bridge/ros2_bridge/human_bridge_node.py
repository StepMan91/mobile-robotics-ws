#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import socket
import json
import threading
import time

class HumanBridgeNode(Node):
    def __init__(self):
        super().__init__('human_bridge_node')
        self.publisher_ = self.create_publisher(MarkerArray, 'human_skeleton', 10)
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
                json_str = data.decode('utf-8')
                self.process_data(json_str)
            except socket.timeout:
                continue
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

        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')

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
