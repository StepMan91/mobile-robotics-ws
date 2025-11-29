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
# This is required because the pip install doesn't always set up the paths correctly for all extensions
# We try to find the extscache directory relative to the isaacsim package
import isaacsim
isaacsim_path = os.path.dirname(isaacsim.__file__)
extscache_path = os.path.join(isaacsim_path, "extscache")

# If not found there, try the hardcoded path from the error log (miniconda env)
if not os.path.exists(extscache_path):
    extscache_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache"

# TARGETED FIX: Add specific dependencies that are known to cause DLL issues
# We avoid adding EVERYTHING to prevent WinError 206 (Filename too long) or PATH overflow

def add_extension_to_path(pattern):
    dirs = glob.glob(os.path.join(extscache_path, pattern))
    for d in dirs:
        # Add bin
        bin_path = os.path.join(d, "bin")
        if os.path.exists(bin_path):
            try:
                os.add_dll_directory(bin_path)
                os.environ['PATH'] = bin_path + os.pathsep + os.environ['PATH']
            except Exception as e:
                print(f"Warning: Failed to add {bin_path}: {e}")
        
        # Add libs
        libs_path = os.path.join(d, "libs")
        if os.path.exists(libs_path):
            try:
                os.add_dll_directory(libs_path)
                os.environ['PATH'] = libs_path + os.pathsep + os.environ['PATH']
            except Exception as e:
                print(f"Warning: Failed to add {libs_path}: {e}")
        
        # Add the dir itself to sys.path for python imports
        if d not in sys.path:
            sys.path.append(d)

print("Applying TARGETED DLL fix...")

# 1. omni.usd and libs
add_extension_to_path("omni.usd-1.*")
add_extension_to_path("omni.usd.libs-*")

# 2. omni.kit.usd
add_extension_to_path("omni.kit.usd.layers-*")
add_extension_to_path("omni.kit.usd.collect-*")

# 3. omni.hydra (Fix for scene_api error)
add_extension_to_path("omni.hydra.scene_api-*")
add_extension_to_path("omni.hydra.usd-*") # Good measure
add_extension_to_path("omni.hydra.rtx-*") # Good measure

# 4. Main Isaac Sim bin
isaacsim_bin = os.path.join(isaacsim_path, "bin")
if os.path.exists(isaacsim_bin):
     try:
        os.add_dll_directory(isaacsim_bin)
        os.environ['PATH'] = isaacsim_bin + os.pathsep + os.environ['PATH']
     except Exception:
        pass

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
    "extra_ext_paths": [extscache_path],
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
