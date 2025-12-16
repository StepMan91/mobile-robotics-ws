# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# Defaults
STEP_HEIGHT = 0.15
STEP_DEPTH = 0.25
WIDTH = 1.0

def create_cube(stage, path, position, scale, color):
    # Define Cube Prim
    cube = UsdGeom.Cube.Define(stage, path)
    
    # Transform (Xform API)
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    # UsdGeom.Cube scale is 2 units by default? No, size is 2. 
    # Actually, UsdGeom.Cube does not have a 'size' attribute in older versions, 
    # but typically it's a unit cube. 
    # Better to use scaling.
    # Scale: The visual size is 'size' * scale. 
    # Note: UsdGeom.Cube default size is 2.0.
    # To get exact dimensions [d_x, d_y, d_z], we need to scale by [d_x/2, d_y/2, d_z/2].
    
    # However, simpler to use VisualCuboid logic? No, that failed.
    # Let's rely on scale.
    # Target Scale = dimensions / 2.0
    
    s = Gf.Vec3f(float(scale[0]/2.0), float(scale[1]/2.0), float(scale[2]/2.0))
    UsdGeom.XformCommonAPI(cube).SetScale(s)
    
    # Color
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(color[0], color[1], color[2])])
    
    # Physics Collision (The Nuclear Option)
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    
    # Optional: Rigid Body (Static)
    # rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    # rb.CreateKinematicEnabledAttr(True) 
    
    return cube

def create_industrial_stairs(world, position, num_steps=15, step_height=STEP_HEIGHT, step_depth=STEP_DEPTH, width=WIDTH):
    """
    Creates stairs using Raw USD Geometry + Physics Schema.
    """
    base_pos = np.array(position)
    stage = omni.usd.get_context().get_stage()
    
    # --- STAIRS ---
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        
        pos = base_pos + np.array([x_offset, 0, z_offset])
        prim_path = f"/World/Stairs/Step_{i}"
        
        create_cube(
            stage, 
            prim_path, 
            pos, 
            scale=[step_depth, width, step_height], 
            color=[0.3, 0.3, 0.35]
        )
        
    # --- CATWALK ---
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([
        (num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth),
        0, 
        (num_steps - 1) * step_height + (step_height / 2.0)
    ])
    
    create_cube(
        stage, 
        "/World/Stairs/Catwalk", 
        catwalk_pos, 
        scale=[catwalk_depth, width, step_height], 
        color=[0.25, 0.25, 0.3]
    )

    # --- HANDRAILS (Simplifying to Boxes for now to ensure collision first) ---
    # Create Posts and Rails
    rail_height = 0.9 
    post_width = 0.04 # Square posts for guaranteed box collision
    rail_width = 0.05
    
    # Calculate Diagonal
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2)
    angle_rad = math.atan2(total_rise, total_run)
    pitch_deg = 90 - math.degrees(angle_rad) # Not needed if we position start/end?
    
    # We will skip complex rotation logic for cylinders and use "Approximated" rails
    # or just posts for collision testing.
    # Actually, let's just make the posts solid cubes.
    
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height 
    
    rail_offsets_y = [width/2.0, -width/2.0]

    for idx, y_off in enumerate(rail_offsets_y):
        # 1. Main Rail (Rotated Cube)
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        # Rotation logic with UsdGeom is XformOp:rotateY
        # Skipping simplified: Just add POSTS.
        
        # 2. Vertical Posts
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height 
             
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             post_path = f"/World/Stairs/Post_{idx}_{p_idx}"
             
             create_cube(
                stage,
                post_path, 
                post_pos,
                scale=[post_width, post_width, rail_height],
                color=[0.2, 0.2, 0.2]
             )

def create_scene():
    stage = omni.usd.get_context().get_stage()
    
    # Physics Scene
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    
    # Ground Plane
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Plane.Define(stage, "/World/GroundPlane").GetPrim())
    
    # Stairs
    create_industrial_stairs(None, position=[2.0, 2.0, 0.0])
    
    # Save
    import os
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/stairs_env.usd"))
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    print(f"Saving stage to {save_path}...")
    omni.usd.get_context().save_as_stage(save_path)
    print("Done.")

import traceback

if __name__ == "__main__":
    try:
        print("[INFO] Starting Export (Raw USD Mode)...")
        create_scene()
        print("[INFO] Export Finished Successfully.")
    except Exception:
        print("[FATAL] Export Failed:")
        traceback.print_exc()
    finally:
        simulation_app.close()
