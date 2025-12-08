from isaacsim import SimulationApp

# 1. Launch Isaac Sim (Disable signal handlers to avoid ROS2 conflict)
simulation_app = SimulationApp({"headless": False, "install_signal_handlers": False})

import omni
import carb
import sys
import os

# New Isaac Sim 5.1 Imports
from isaacsim.core.api.world import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import is_prim_path_valid

# 2. PATCH ENVIRONMENT FOR ROS2 BRIDGE (CRITICAL FIX)
# Isaac Sim 5.1 bundled ROS2 needs specific env vars and DLL search paths
# set BEFORE enabling the extension.
def patch_ros2_env():
    print("[Viz] Patching environment for bundles ROS2 Bridge...")
    
    # Paths for Isaac Sim 5.1 / Windows
    bridge_base = r"C:\isaac-sim\exts\isaacsim.ros2.bridge\humble"
    libs_path = os.path.join(bridge_base, "lib")
    rclpy_path = os.path.join(bridge_base, "rclpy")
    
    # 1. Set ROS Env Vars
    os.environ["ROS_DISTRO"] = "humble"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    
    # 2. Add Libs to PATH (for LoadLibrary)
    current_path = os.environ.get("PATH", "")
    if libs_path not in current_path:
        print(f"[Viz] Adding {libs_path} to PATH")
        os.environ["PATH"] = libs_path + os.pathsep + current_path
        
    # 3. Add DLL Directory (Python 3.8+ specific)
    if hasattr(os, "add_dll_directory") and os.path.isdir(libs_path):
        try:
            os.add_dll_directory(libs_path)
            print(f"[Viz] Added DLL directory: {libs_path}")
        except Exception as e:
            print(f"[Viz] WARNING: Failed to add DLL directory: {e}")

    # 4. Add rclpy to sys.path
    if rclpy_path not in sys.path:
        sys.path.append(rclpy_path)
        print(f"[Viz] Added {rclpy_path} to sys.path")

patch_ros2_env()

# 3. Enable ROS2 Bridge Extension explicitly BEFORE importing rclpy
print("[Viz] Enabling ROS2 Bridge Extension...")
try:
    enable_extension("isaacsim.ros2.bridge")
    print("[Viz] Bridge Extension Enabled.")
except Exception as e:
    print(f"[Viz] CRITICAL ERROR: Failed to enable ROS2 Bridge: {e}")

# 4. Import ROS2 modules (Now safe)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    print("[Viz] ROS2 Modules Imported Successfully.")
except ImportError as e:
    print(f"[Viz] CRITICAL ERROR: Failed to import ROS2 modules: {e}")
    sys.exit(1)

import numpy as np

# 5. Define Visualizer Node (now Node is defined)
class VisualizerNode(Node):
    def __init__(self, robot_prim):
        super().__init__('isaac_g1_viz')
        self.robot = robot_prim
        self.joint_map = {} # ROS Name -> DOF Index
        
        # Quality of Life: Log startup
        self.get_logger().info("G1 Visualizer Node Started")
        
        self.sub_js = self.create_subscription(JointState, 'joint_states', self.js_callback, 10)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
        self.base_pos = np.array([0.0, 0.0, 0.82]) # Default standing height
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
    
    # Path to G1 USD
    usd_path = r"c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
    prim_path = "/World/G1"
    
    print(f"[Viz] Loading USD from: {usd_path}")
    
    # Add Ground Plane
    world.scene.add_default_ground_plane()
    
    # Load USD
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    
    # Verify Prim Exists
    if not is_prim_path_valid(prim_path):
        print(f"[Viz] ERROR: Prim {prim_path} not valid after loading USD!")
    else:
        print(f"[Viz] SUCCESS: Prim {prim_path} loaded successfully.")

    # Initialize Robot Wrapper
    robot = Robot(prim_path=prim_path, name="g1")
    world.scene.add(robot)
    
    world.reset()
    print("[Viz] World Reset Complete.")

    # Init ROS
    rclpy.init()
    node = VisualizerNode(robot)
    
    print("[Viz] Starting loop...")
    
    # Force first render
    world.step(render=True)
    
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(node, timeout_sec=0.001)
        
        # Explicit update Base Pose
        robot.set_world_pose(position=node.base_pos, orientation=node.base_rot)

    simulation_app.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
