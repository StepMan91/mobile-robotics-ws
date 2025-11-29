# Isaac Sim Script - Stair Environment & Robot Loader
# Run this in the Isaac Sim Script Editor or via python.sh

import omni
from pxr import Usd, UsdGeom, Gf, Sdf
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import numpy as np

def create_stairs(stage_path="/World/Stairs", num_steps=10, step_width=2.0, step_height=0.15, step_depth=0.3):
    """Generates a procedural staircase."""
    for i in range(num_steps):
        prim_path = f"{stage_path}/Step_{i}"
        
        # Position: Each step is higher and further forward
        position = np.array([i * step_depth, 0, i * step_height])
        
        # Create a cube for the step
        prim_utils.create_prim(
            prim_path=prim_path,
            prim_type="Cube",
            position=position,
            scale=np.array([step_depth, step_width, step_height]),
            attributes={"displayColor": [(0.5, 0.5, 0.5)]}
        )
        # Add collision
        # (In a real script we would add PhysicsCollisionAPI here)

def load_robot(usd_path, prim_path="/World/Unitree_G1"):
    """Loads the Unitree G1 robot from USD."""
    if not usd_path:
        print("Error: No USD path provided for robot.")
        return

    prim_utils.create_prim(
        prim_path=prim_path,
        usd_path=usd_path,
        position=np.array([-1.0, 0, 0.5]) # Start before the stairs
    )

def main():
    # 1. Setup Stage
    stage_utils.create_new_stage()
    stage_utils.add_reference_to_stage(
        usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Grid/default_environment.usd",
        prim_path="/World/GroundPlane"
    )

    # 2. Create Stairs
    create_stairs(num_steps=15, step_height=0.18, step_depth=0.28)

    # 3. Load Robot (Placeholder path - User needs to update this)
    # TODO: Update this path to the actual G1 USD file location
    g1_usd_path = "C:/Users/basti/Documents/Unitree/g1_edu.usd" 
    load_robot(g1_usd_path)

    print("Environment Generated.")

if __name__ == "__main__":
    main()
