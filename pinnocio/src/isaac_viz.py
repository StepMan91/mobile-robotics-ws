from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni
import carb
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.extensions import enable_extension
from omni.importer.urdf import _urdf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import os

class VisualizerNode(Node):
    def __init__(self, robot_prim):
        super().__init__('isaac_g1_viz')
        self.robot = robot_prim
        self.joint_map = {} # ROS Name -> DOF Index
        
        self.sub_js = self.create_subscription(JointState, 'joint_states', self.js_callback, 10)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
        self.base_pos = np.array([0.0, 0.0, 0.0])
        self.base_rot = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z

    def js_callback(self, msg):
        # Map ROS joints to Isaac Dof indices
        if not self.joint_map:
             # Lazy init
             dof_names = self.robot.dof_names
             for i, name in enumerate(dof_names):
                 self.joint_map[name] = i
        
        indices = []
        positions = []
        
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                indices.append(self.joint_map[name])
                positions.append(msg.position[i])
        
        if indices:
            self.robot.set_joint_positions(positions=np.array(positions), joint_indices=np.array(indices))

    def tf_callback(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'base_link' or t.child_frame_id == 'pelvis':
                p = t.transform.translation
                r = t.transform.rotation
                self.base_pos = np.array([p.x, p.y, p.z])
                self.base_rot = np.array([r.w, r.x, r.y, r.z])

def main():
    world = World()
    
    # Import URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.abspath(os.path.join(script_dir, "../models/g1_description/urdf/g1.urdf"))
    
    # Configure URDF Importer
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = True
    
    # Import
    prim_path = urdf_interface.import_robot(urdf_path, "/World/G1", import_config)
    
    robot = Robot(prim_path=prim_path, name="g1")
    world.scene.add(robot)
    
    world.reset()
    
    # Init ROS
    rclpy.init()
    node = VisualizerNode(robot)
    
    print("[Viz] Starting loop...")
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(node, timeout_sec=0.001)
        
        # Explicit update Base Pose
        # Note: set_world_pose works on the root prim.
        # If URDF imported with a fixed root joint, this might fight physics.
        # But we set fix_base=False
        robot.set_world_pose(position=node.base_pos, orientation=node.base_rot)

    simulation_app.close()

if __name__ == "__main__":
    main()
