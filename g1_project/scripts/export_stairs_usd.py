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

def create_primitive(stage, prim_type, path, position, scale=None, radius=None, height=None, color=None):
    # Geometry
    if prim_type == "Cube":
        prim_geom = UsdGeom.Cube.Define(stage, path)
        if scale:
             s = Gf.Vec3f(scale[0]/2.0, scale[1]/2.0, scale[2]/2.0)
             UsdGeom.XformCommonAPI(prim_geom).SetScale(s)
    elif prim_type == "Cylinder":
        prim_geom = UsdGeom.Cylinder.Define(stage, path)
        if radius:
             prim_geom.GetRadiusAttr().Set(float(radius))
        if height:
             prim_geom.GetHeightAttr().Set(float(height))
        # Cylinders in USD align along Z by default. 
        # If we need rotation, we handle it in Xform.

    # Transform
    UsdGeom.XformCommonAPI(prim_geom).SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    
    # Color
    if color:
        prim_geom.GetDisplayColorAttr().Set([Gf.Vec3f(color[0], color[1], color[2])])
    
    # PHYSICS COLLISION
    prim = prim_geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    
    # FORCE KINEMATIC (So RayCaster sees it moving/static correctly)
    rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb_api.CreateKinematicEnabledAttr(True)
    
    return prim_geom

def create_industrial_stairs(world, position, num_steps=15, step_height=STEP_HEIGHT, step_depth=STEP_DEPTH, width=WIDTH):
    base_pos = np.array(position)
    stage = omni.usd.get_context().get_stage()
    
    # 0. BLUE RUNWAY (The "Carpet" in the image)
    # 2m long, same width as stairs, in front of first step
    runway_len = 3.0
    runway_pos = base_pos + np.array([-runway_len/2.0 - step_depth/2.0, 0, 0.05]) # Slightly up to avoid Z-fight
    create_primitive(stage, "Cube", "/World/Stairs/Runway",
                     position=runway_pos,
                     scale=[runway_len, width, 0.1],
                     color=[0.0, 0.5, 0.9]) # Blue
    
    # --- STAIRS ---
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        
        pos = base_pos + np.array([x_offset, 0, z_offset])
        prim_path = f"/World/Stairs/Step_{i}"
        
        create_primitive(stage, "Cube", prim_path, 
            pos, 
            scale=[step_depth, width, step_height], 
            color=[0.6, 0.6, 0.65] # Grey
        )
        
    # --- CATWALK ---
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([
        (num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth),
        0, 
        (num_steps - 1) * step_height + (step_height / 2.0)
    ])
    create_primitive(stage, "Cube", "/World/Stairs/Catwalk",
        catwalk_pos, 
        scale=[catwalk_depth, width, step_height], 
        color=[0.6, 0.6, 0.65]
    )

    # --- HANDRAILS (Yellow Cylinders) ---
    rail_height = 0.9 
    rail_radius = 0.04
    rail_offsets_y = [width/2.0, -width/2.0]
    
    # Calculate Diagonal Length & Angle
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2) + 0.5 # Extra bit
    angle_rad = math.atan2(total_rise, total_run)
    pitch_deg = -math.degrees(angle_rad) # Rotate DOWN (Negative Y rotation?) 
    # UsdGeom Cylinder is Z-up. 
    # To align with stairs (X-up slope), we need to RotateY.
    # Actually, simpler: Rotate around Y axis.
    # 0 deg = Vertical. 90 deg = Horizontal X.
    # Slope is roughly 30-40 deg from Horizontal.
    # So rotation is 90 - slope? Or just Euler angles.
    
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height/2.0
    
    for idx, y_off in enumerate(rail_offsets_y):
        # 1. DIAGONAL RAIL
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        rail_path = f"/World/Stairs/Rail_{idx}"
        
        prim = create_primitive(stage, "Cylinder", rail_path,
                         position=rail_pos,
                         radius=rail_radius,
                         height=diag_len,
                         color=[0.9, 0.8, 0.2]) # Yellow
        
        # Rotate: Cylinder is Z-aligned.
        # We want it tilted in X-Z plane.
        # Rotate Y.
        # Angle: Z-axis -> Slope.
        # Slope angle from Horiz = atan(rise/run).
        # Cylinder Z is Vert.
        # Rotate Y by 90 + Slope?
        # Let's try 90 - 30 = 60?
        # Slope angle ~ 30 deg.
        rotation_angle = 90 - math.degrees(angle_rad) 
        UsdGeom.XformCommonAPI(prim).SetRotate(Gf.Vec3f(0.0, rotation_angle, 0.0))

        # 2. VERTICAL POSTS
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height 
             
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             post_path = f"/World/Stairs/Post_{idx}_{p_idx}"
             
             create_primitive(stage, "Cylinder", post_path,
                position=post_pos,
                radius=rail_radius,
                height=rail_height,
                color=[0.9, 0.8, 0.2] # Yellow
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
    create_industrial_stairs(None, position=[2.0, 0.0, 0.0])
    
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
