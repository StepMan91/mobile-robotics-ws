# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, UsdLux

# Defaults
STEP_HEIGHT = 0.15
STEP_DEPTH = 0.25
WIDTH = 1.0

def create_cube(stage, path, position, scale, color):
    # Define Cube Prim
    cube = UsdGeom.Cube.Define(stage, path)
    
    # Scale
    s = Gf.Vec3f(scale[0]/2.0, scale[1]/2.0, scale[2]/2.0)
    UsdGeom.XformCommonAPI(cube).SetScale(s)
    
    # Transform
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    
    # Color
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(color[0], color[1], color[2])])
    
    # PHYSICS COLLISION (The Nuclear Option)
    prim = cube.GetPrim()
    
    # 1. Collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    collision_api.CreateCollisionEnabledAttr(True)
    
    # 2. Rigid Body API (Kinematic)
    # Forces PhysX to track the object
    rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb_api.CreateKinematicEnabledAttr(True)
    
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

    # --- HANDRAILS ---
    # Create Posts (Square - Robust Collision)
    rail_height = 0.9 
    post_width = 0.04
    rail_offsets_y = [width/2.0, -width/2.0]
    
    # Vertical Posts
    post_indices = [0, num_steps // 2, num_steps - 1]
    for idx, y_off in enumerate(rail_offsets_y):
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
    
     # --- LIGHTING REMOVED (Handled by Env Config) ---
    
    # Ground Plane - REMOVED (Handled by TerrainGenerator)
    
    # Stairs
    create_industrial_stairs(None, position=[2.0, 2.0, 0.0])
    
    # Save
    import os
    import time
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/stairs_env.usd"))
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    print(f"Saving stage to {save_path}...")
    
    # Synchronous Export
    stage.GetRootLayer().Export(save_path)
    
    # Wait and Verify
    time.sleep(1.0)
    if os.path.exists(save_path):
        print(f"[SUCCESS] File created at: {save_path}")
        print(f"Size: {os.path.getsize(save_path)} bytes")
    else:
        print(f"[ERROR] File NOT created at: {save_path}")
        
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
