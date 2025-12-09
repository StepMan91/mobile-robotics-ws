#!/usr/bin/env python3
"""
G1 Robot State Publisher for RViz2
Publishes robot_description and transforms the robot state from joint_states to TF
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import os


class G1StatePublisher(Node):
    def __init__(self):
        super().__init__('g1_state_publisher')
        
        # Publisher for robot description
        self.desc_pub = self.create_publisher(String, '/robot_description', 10)
        
        # Relay joint states (from main.py retargeter)
        self.js_sub = self.create_subscription(
            JointState, 
            'joint_states', 
            self.joint_state_callback, 
            10
        )
        
        # Relay TF data
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        # Republish for robot_state_publisher compatibility
        self.js_pub = self.create_publisher(JointState, '/joint_states_relay', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_relay', 10)
        
        # Publish robot description once
        self.publish_robot_description()
        
        self.get_logger().info("G1 State Publisher Started - Ready for RViz2")
    
    def publish_robot_description(self):
        """Load and publish the G1 URDF"""
        urdf_path = r"C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\src\g1_description\urdf\g1.urdf"
        
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF not found: {urdf_path}")
            return
        
        try:
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
            
            msg = String()
            msg.data = urdf_content
            
            # Publish robot description (latched topic)
            for _ in range(5):  # Publish multiple times to ensure it's received
                self.desc_pub.publish(msg)
            
            self.get_logger().info(f"Published robot_description ({len(urdf_content)} bytes)")
        except Exception as e:
            self.get_logger().error(f"Failed to publish robot description: {e}")
    
    def joint_state_callback(self, msg):
        """Relay joint states"""
        self.js_pub.publish(msg)
    
    def tf_callback(self, msg):
        """Relay TF transforms"""
        self.tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = G1StatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
