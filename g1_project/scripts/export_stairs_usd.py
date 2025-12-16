# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
import math
from isaacsim.core.api.world import World
from isaacsim.core.api.objects import VisualCuboid, VisualCylinder
from pxr import UsdPhysics, Gf

# Defaults
STEP_HEIGHT = 0.15
STEP_DEPTH = 0.25
WIDTH = 1.0

def create_industrial_stairs(world, position, num_steps=15, step_height=STEP_HEIGHT, step_depth=STEP_DEPTH, width=WIDTH):
    """
    Creates an industrial-style staircase and a catwalk with handrails.
    Uses Explicit Physics APIs to guarantee collisions.
    """
    base_pos = np.array(position)
    stage = omni.usd.get_context().get_stage()

    def apply_physics(prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        # 1. Collision API (Basic)
        UsdPhysics.CollisionAPI.Apply(prim)
        # 2. Mesh Collision API (Forcing mesh-based check if prim is mesh)
        # UsdPhysics.MeshCollisionAPI.Apply(prim) # Optional, usually CollisionAPI is enough for Cubes
        # 3. Rigid Body API (Kinematic - Static but tracked)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(True)
    
    # --- STAIRS ---
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        
        pos = base_pos + np.array([x_offset, 0, z_offset])
        prim_path = f"/World/Stairs/Step_{i}"
        
        world.scene.add(
            VisualCuboid(
                prim_path=prim_path,
                name=f"step_{i}",
                position=pos,
                scale=np.array([step_depth, width, step_height]),
                color=np.array([0.3, 0.3, 0.35]) 
            )
        )
        apply_physics(prim_path)
        
    # --- CATWALK ---
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([
        (num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth),
        0, 
        (num_steps - 1) * step_height + (step_height / 2.0)
    ])
    
    catwalk_path = "/World/Stairs/Catwalk"
    world.scene.add(
        VisualCuboid(
            prim_path=catwalk_path,
            name="catwalk",
            position=catwalk_pos,
            scale=np.array([catwalk_depth, width, step_height]),
            color=np.array([0.25, 0.25, 0.3])
        )
    )
    apply_physics(catwalk_path)

    # --- HANDRAILS ---
    # Create Posts and Rails
    rail_height = 0.9 # Standard
    post_radius = 0.02
    rail_radius = 0.025
    
    # Calculate Diagonal Length and Angle
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2)
    angle_rad = math.atan2(total_rise, total_run)
    
    # Center of diagonal rail
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height # Offset up
    
    rail_offsets_y = [width/2.0, -width/2.0]
    
    for idx, y_off in enumerate(rail_offsets_y):
        # 1. Main Diagonal Rail
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        
        pitch_deg = 90 - math.degrees(angle_rad) 
        
        rad = math.radians(pitch_deg)
        orient = np.array([math.cos(rad/2), 0, math.sin(rad/2), 0])
        
        rail_path = f"/World/Stairs/Rail_Diag_{idx}"
        world.scene.add(
            VisualCylinder(
                prim_path=rail_path,
                name=f"rail_diag_{idx}",
                position=rail_pos,
                scale=np.array([rail_radius, rail_radius, diag_len + 0.5]), 
                color=np.array([0.8, 0.8, 0.2]), # Yellow/Safety
                orientation=orient 
            )
        )
        apply_physics(rail_path)
        
        # 2. Vertical Posts (Start, Middle, End)
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height # On Step Surface
             
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             post_path = f"/World/Stairs/Post_{idx}_{p_idx}"
             
             world.scene.add(
                VisualCylinder(
                    prim_path=post_path,
                    name=f"post_{idx}_{p_idx}",
                    position=post_pos,
                    scale=np.array([post_radius, post_radius, rail_height]),
                    color=np.array([0.2, 0.2, 0.2])
                )
             )
             apply_physics(post_path)

def create_scene():
    world = World()
    
    # Ground Plane
    world.scene.add_default_ground_plane()

    # --- PHYSICS SCENE (Crucial for Collisions) ---
    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    # ----------------------------------------------
    
    # Stairs
    # Use position from Pantin/visualize_csv_isaac.py: [2.0, 2.0, 0.0]
    create_industrial_stairs(world, position=[2.0, 2.0, 0.0])
    
    # Save
    import os
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/stairs_env.usd"))
    # Ensure dir
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    # Export
    print(f"Saving stage to {save_path}...")
    omni.usd.get_context().save_as_stage(save_path)
    print("Done.")

import traceback

if __name__ == "__main__":
    try:
        print("[INFO] Starting Export...")
        create_scene()
        print("[INFO] Export Finished Successfully.")
    except Exception:
        print("[FATAL] Export Failed:")
        traceback.print_exc()
    finally:
        simulation_app.close()
