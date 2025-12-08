from isaacsim import SimulationApp

# Headless mode for automated verification
simulation_app = SimulationApp({"headless": True, "install_signal_handlers": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
# from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import is_prim_path_valid

def test_main():
    print("[Test] Starting Headless Verification (Clean Environment)...")
    
    # 1. Enable ROS2 Bridge
    # Note: With proper env vars, enable_extension might not be strictly needed if autoload is configured,
    # but we do it to be safe and consistent with isaac_viz.py
    try:
        from omni.isaac.core.utils.extensions import enable_extension
        print("[Test] Enabling isaacsim.ros2.bridge...")
        enable_extension("isaacsim.ros2.bridge")
        print("[Test] Bridge Extension Enabled.")
    except Exception as e:
        print(f"[Test] ERROR enabling extension: {e}")
        # continue anyway to check imports

    # 2. Check ROS2 Imports
    try:
        print("[Test] Importing rclpy...")
        import rclpy
        print(f"[Test] rclpy imported: {rclpy.__file__}")
        
        from sensor_msgs.msg import JointState
        print(f"[Test] sensor_msgs imported.")
        
    except ImportError as e:
        print(f"[Test] ERROR importing rclpy: {e}")
        return
    except Exception as e:
        print(f"[Test] ERROR general: {e}")
        return

    # 3. Load USD
    world = World()
    usd_path = r"c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
    prim_path = "/World/G1"
    
    print(f"[Test] Loading USD: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    
    if is_prim_path_valid(prim_path):
        print("[Test] SUCCESS: USD loaded and Prim found.")
    else:
        print("[Test] ERROR: Prim not found after loading.")
        
    world.reset()
    print("[Test] World Reset. Verification Complete.")
    
    simulation_app.close()

if __name__ == "__main__":
    test_main()
