import os
import sys

# Use AppLauncher to start the app, as it handles environment setup correctly
from isaaclab.app import AppLauncher

import argparse

# Define arguments for AppLauncher
args = argparse.Namespace(headless=True, livestream=0)

# Initialize AppLauncher
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

print("SimulationApp started successfully via AppLauncher.")

# Now import Isaac Lab tools
from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg

# Define paths
# Use absolute paths to be safe
root_dir = r"c:\Users\basti\.gemini\antigravity\playground\cobalt-cosmos"
input_urdf = os.path.join(root_dir, "assets/unitree_ros/robots/g1_description/g1_29dof.urdf")
output_usd = os.path.join(root_dir, "assets/g1_29dof.usd")

print(f"Converting: {input_urdf}")
print(f"To: {output_usd}")

# Configure converter
cfg = UrdfConverterCfg(
    asset_path=input_urdf,
    usd_dir=os.path.dirname(output_usd),
    usd_file_name=os.path.basename(output_usd),
    fix_base=False,
    merge_fixed_joints=False,
    force_usd_paths_as_relative=True,
)

converter = UrdfConverter(cfg)
converter.convert()

print("Conversion finished!")
simulation_app.close()
