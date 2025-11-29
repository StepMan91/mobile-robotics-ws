import argparse
import os
import sys

# Argument parsing
parser = argparse.ArgumentParser(description="Verify G1 Asset in Isaac Sim")
parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode")
args = parser.parse_args()

# --- ISAAC SIM SETUP ---
import glob

# WORKAROUND: Add Isaac Sim extensions to sys.path manually
# Even with a proper install, standalone scripts sometimes miss these paths
import isaacsim
isaacsim_path = os.path.dirname(isaacsim.__file__)
extscache_path = os.path.join(isaacsim_path, "extscache")

# Helper to add extension to sys.path
def add_ext_to_path(pattern):
    dirs = glob.glob(os.path.join(extscache_path, pattern))
    if dirs:
        sys.path.append(dirs[0])

# Add critical extensions to sys.path
add_ext_to_path("omni.kit.usd.layers-*")
add_ext_to_path("omni.usd-1.*")
add_ext_to_path("omni.kit.usd.collect-*")
add_ext_to_path("omni.usd.libs-*")
add_ext_to_path("omni.hydra*")

from isaacsim import SimulationApp

# Configure the simulation app
config = {
    "headless": args.headless,
    "width": 1280,
    "height": 720,
    "window_width": 1280,
    "window_height": 720,
    "renderer": "RayTracedLighting",
    "display_options": 3286,  # Show Grid
}

# Start the app
simulation_app = SimulationApp(config)
print("SimulationApp started successfully.")

# --- IMPORTS AFTER STARTUP ---
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core.utils.viewports import set_camera_view

# --- MAIN LOGIC ---
def main():
    # Define path to USD
    # Use absolute path relative to this script to be robust
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up two levels: scripts -> g1_project -> mobile-robotics-ws
    workspace_root = os.path.abspath(os.path.join(script_dir, "../../"))
    usd_path = os.path.join(workspace_root, "assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd")

    print(f"Workspace Root: {workspace_root}")
    print(f"Target USD: {usd_path}")

    if not os.path.exists(usd_path):
        print(f"[ERROR] File not found at: {usd_path}")
        simulation_app.close()
        return

    try:
        # Open the stage
        print(f"Opening stage...")
        open_stage(usd_path)
        print("Stage opened successfully.")

        # Create a ground plane if it doesn't exist (optional, for better view)
        # prim_utils.create_prim("/World/GroundPlane", "Plane", translation=(0, 0, 0), scale=(10, 10, 1))

        # Set camera view to look at the robot
        set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.8], camera_prim_path="/OmniverseKit_Persp")

        print("Asset loaded. Running simulation loop...")
        print("Press Ctrl+C in terminal or close the window to exit.")

        while simulation_app.is_running():
            simulation_app.update()

    except Exception as e:
        print(f"[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        simulation_app.close()

if __name__ == "__main__":
    main()
