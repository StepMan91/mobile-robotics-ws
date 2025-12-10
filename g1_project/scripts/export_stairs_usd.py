
import omni.kit.commands
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, UsdLux
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

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
        
        # Create Visual Step
        omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
            prim_type='Cube',
            prim_path=step_path)
            
        prim = stage.GetPrimAtPath(step_path)
        
        # Transform (Scale to step shape)
        # Cube default is 1.0 size (-0.5 to 0.5)
        # We want X=depth, Y=width=1.0, Z=height
        
        # Scale
        # Note: Scale is applied to unit cube.
        # We want width = 1.0m (Standard).
        
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xform.AddScaleOp().Set(Gf.Vec3d(step_depth, 1.0, step_height))
        
        # Collision
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # Material
        omni.kit.commands.execute('BindMaterial',
            prim_path=step_path,
            material_path=mat_path)
            
    # Platform at top
    plat_path = f"{base_path}/TopPlatform"
    px = position[0] + num_steps * step_depth + (1.0) # 2m deep platform center? No
    # Last step end X = start + num*depth.
    # Platform center X = end + plat_depth/2 - step_depth/2?
    # Simpler:
    last_x = position[0] + (num_steps-1) * step_depth
    last_z = position[2] + (num_steps-1) * step_height + (step_height/2)
    
    plat_depth = 2.0
    plat_z = last_z # Same height as last step? Or one higher? 
    # Usually platform is the Nth step.
    # Let's say top platform continues from last step level.
    
    # Actually create_industrial_stairs in climb_stairs.py seemingly made a catwalk.
    # Let's use simple logic: Platform is next step.
    
    # We want a 2m long platform.
    # Center X = (Start of Platform) + 1.0
    # Start of Platform = (Last Step Center) + Depth/2?
    plat_center_x = (position[0] + num_steps * step_depth) + (plat_depth / 2.0) - (step_depth / 2.0) 
    # This aligns the edge.
    
    omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
        prim_type='Cube',
        prim_path=plat_path)
    prim = stage.GetPrimAtPath(plat_path)
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(plat_center_x, position[1], last_z))
    xform.AddScaleOp().Set(Gf.Vec3d(plat_depth, 1.0, step_height))
    UsdPhysics.CollisionAPI.Apply(prim)
    omni.kit.commands.execute('BindMaterial', prim_path=plat_path, material_path=mat_path)
    
    # HANDRAIL
    # Create simple cylinder handle
    rail_path = "/World/Handrail"
    omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
        prim_type='Cylinder',
        prim_path=rail_path)
    
    # Geometry:
    # Start: (3.0, 0.6, 1.0) # approx
    # End: Top of stairs.
    # Slope calculation.
    run = num_steps * step_depth
    rise = num_steps * step_height
    length = np.sqrt(run**2 + rise**2) + 2.0 # Extend past top
    angle = np.arctan2(rise, run)
    
    # Center position
    cx = position[0] + run/2.0
    cz = position[2] + rise/2.0 + 0.9 # 0.9m rail height
    cy = position[1] + 0.45 # Check side? Width 1.0 -> +/- 0.5. Rail at 0.45 is good.
    
    prim = stage.GetPrimAtPath(rail_path)
    xform = UsdGeom.Xformable(prim)
    
    # Rotate pitch (Y axis rotation)
    # Default cylinder is along Z? or Y? Usually Y or Z.
    # If Z, we need to rotate around Y.
    # Check default axis. 
    # Let's generic rotate.
    
    # Rotate -angle degrees around Y (Pitch up).
    # Convert to degrees
    deg = -np.degrees(angle)
    
    # But Cylinder default orientation usually Z-up. 
    # To make it "forward" (X), we rotate 90 deg Y.
    # Combined: 90 + angle?
    
    # Proper transform:
    # 1. Scale length (Height in Z) -> length
    # 2. Radius -> 0.02
    # 3. Rotate to slope.
    
    # Or just use Xform:
    xform.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xform.AddRotateYOp().Set(90 + deg) # Check signs
    xform.AddScaleOp().Set(Gf.Vec3d(0.04, 0.04, length)) # Radius 2cm
    
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

if __name__ == "__main__":
    create_scene()
    simulation_app.close()
