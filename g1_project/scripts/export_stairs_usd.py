# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.kit.commands
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, UsdLux
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

def create_industrial_stairs(world, position, num_steps=15, step_height=0.15, step_depth=0.25):
    stage = get_current_stage()
    base_path = "/World/Stairs"
    
    # Material
    mat_path = "/World/Materials/ChequerPlate"
    omni.kit.commands.execute('CreateMdlMaterialPrim',
        mtl_url='http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Metals/Checker_Plate.mdl',
        mtl_name='Checker_Plate',
        mtl_path=mat_path)

    for i in range(num_steps):
        step_path = f"{base_path}/Step_{i}"
        
        # Calculate pos
        x = position[0] + i * step_depth
        z = position[2] + i * step_height + (step_height/2)
        y = position[1]
        
        # Create Visual Step using simple definition to avoid Auto-Xform confusion
        # Or Just use default and XformCommonAPI
        omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
            prim_type='Cube',
            prim_path=step_path)
            
        prim = stage.GetPrimAtPath(step_path)
        
        # Use XformCommonAPI to set transform safely
        xform_api = UsdGeom.XformCommonAPI(prim)
        # Note: SetTranslate, SetScale, SetRotate
        xform_api.SetTranslate(Gf.Vec3d(x, y, z))
        xform_api.SetScale(Gf.Vec3f(step_depth, 1.0, step_height))
        
        # Collision
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # Material
        omni.kit.commands.execute('BindMaterial',
            prim_path=step_path,
            material_path=mat_path)
            
    # Platform at top
    plat_path = f"{base_path}/TopPlatform"
    last_z = position[2] + (num_steps-1) * step_height + (step_height/2)
    plat_depth = 2.0
    # Center X
    plat_center_x = (position[0] + num_steps * step_depth) + (plat_depth / 2.0) - (step_depth / 2.0) 
    
    omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
        prim_type='Cube',
        prim_path=plat_path)
    prim = stage.GetPrimAtPath(plat_path)
    
    xform_api = UsdGeom.XformCommonAPI(prim)
    xform_api.SetTranslate(Gf.Vec3d(plat_center_x, position[1], last_z))
    xform_api.SetScale(Gf.Vec3f(plat_depth, 1.0, step_height))
    
    UsdPhysics.CollisionAPI.Apply(prim)
    omni.kit.commands.execute('BindMaterial', prim_path=plat_path, material_path=mat_path)
    
    # HANDRAIL
    rail_path = "/World/Handrail"
    omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
        prim_type='Cylinder',
        prim_path=rail_path)
    
    # Geometry:
    run = num_steps * step_depth
    rise = num_steps * step_height
    length = np.sqrt(run**2 + rise**2) + 2.0 
    angle = np.arctan2(rise, run)
    
    # Center position
    cx = position[0] + run/2.0
    cz = position[2] + rise/2.0 + 0.9 
    cy = position[1] + 0.45 
    
    prim = stage.GetPrimAtPath(rail_path)
    xform_api = UsdGeom.XformCommonAPI(prim)
    xform_api.SetTranslate(Gf.Vec3d(cx, cy, cz))
    xform_api.SetScale(Gf.Vec3f(0.04, 0.04, length))
    
    # Rotate: Cylinder is usually Z-Axis aligned.
    # We want to pitch it up around Y axis.
    # XformCommonAPI SetRotate uses (h, p, r) or X,Y,Z euler?
    # It takes Vec3f rotation in degrees (XYZ usually).
    # We want Y rotation of -(90 + angle)?
    # Wait, To make Z axis point along the slope:
    # 1. Rotate Y by 90 (Points Z along X).
    # 2. Rotate Y by -degrees(angle) (Pitch up).
    # Total Y = 90 - degrees(angle).
    
    deg = np.degrees(angle)
    xform_api.SetRotate(Gf.Vec3f(0.0, 90.0 - deg, 0.0))
    
    UsdPhysics.CollisionAPI.Apply(prim)

def create_scene():
    world = World()
    
    # Ground Plane
    world.scene.add_default_ground_plane()
    
    # Stairs
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0])
    
    # Light
    # Using Dome Light from environment cfg usually, but adding one here ensures visibility in raw USD view.
    
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
