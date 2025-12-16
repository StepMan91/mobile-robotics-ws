import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import pinocchio as pin

class Ros2Publisher(Node):
    def __init__(self, model):
        super().__init__('pinnocio_publisher')
        self.model = model
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def publish(self, q):
        # q: [pos(3), quat(4), joints(n)] (if FreeFlyer is first)
        
        now = self.get_clock().now().to_msg()
        
        # 1. Publish TF (Root)
        # Check if first joint is FreeFlyer (7 DoF)
        if self.model.joints[1].shortname() == "JointModelFreeFlyer":
            # q[0:3] = pos, q[3:7] = quat (x,y,z,w)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map" # or world
            t.child_frame_id = "base_link" # or pelvis, whatever is root in URDF
            
            t.transform.translation.x = float(q[0])
            t.transform.translation.y = float(q[1])
            t.transform.translation.z = float(q[2])
            
            t.transform.rotation.x = float(q[3])
            t.transform.rotation.y = float(q[4])
            t.transform.rotation.z = float(q[5])
            t.transform.rotation.w = float(q[6])
            
            self.tf_broadcaster.sendTransform(t)
            
            joint_start_idx = 7
        else:
            joint_start_idx = 0
            
        # 2. Publish Joint States
        msg = JointState()
        msg.header.stamp = now
        
        # Pinocchio stores joint names. We skip "universe" (0) and "root_joint" (1)
        # The q vector corresponds to model.joints. 
        # But model.names has names.
        
        names = []
        positions = []
        
        q_idx = joint_start_idx
        # Iterate over joints starting from 2 (after universe and freeflyer)
        for i in range(2, self.model.njoints):
            joint_name = self.model.names[i]
            # Get nq for this joint
            nq = self.model.joints[i].nq 
            idx_q = self.model.joints[i].idx_q
            
            # Simple 1-DOF joints
            if nq == 1:
                names.append(joint_name)
                positions.append(float(q[idx_q]))
        
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)
