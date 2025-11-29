import os
import argparse
from isaaclab.app import AppLauncher

# Define arguments
args = argparse.Namespace(headless=True, livestream=0)

# Initialize AppLauncher
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

print("SimulationApp started successfully.")

from omni.isaac.core.utils.stage import open_stage, is_stage_loading

# Define path to USD
# Define path to USD
root_dir = r"c:\Users\basti\source\repos\mobile-robotics-ws"
usd_path = os.path.join(root_dir, "assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd")

print(f"Attempting to open stage: {usd_path}")

if not os.path.exists(usd_path):
    print(f"ERROR: File not found at {usd_path}")
    simulation_app.close()
    exit(1)

open_stage(usd_path)

# Wait for loading? usually open_stage is synchronous or we can check
print("Stage opened. Asset loaded successfully!")

simulation_app.close()
print("Verification Complete.")
