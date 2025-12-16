# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, UsdLux

def create_primitive(stage, prim_type, path, position, scale=None, radius=None, height=None, color=None):
    if prim_type == "Cube":
        prim_geom = UsdGeom.Cube.Define(stage, path)
        if scale:
             s = Gf.Vec3f(scale[0]/2.0, scale[1]/2.0, scale[2]/2.0)
             UsdGeom.XformCommonAPI(prim_geom).SetScale(s)
             
    elif prim_type == "Sphere":
        prim_geom = UsdGeom.Sphere.Define(stage, path)
        if radius:
             prim_geom.GetRadiusAttr().Set(float(radius))
             
    elif prim_type == "Cylinder":
        prim_geom = UsdGeom.Cylinder.Define(stage, path)
        if radius:
             prim_geom.GetRadiusAttr().Set(float(radius))
        if height:
             prim_geom.GetHeightAttr().Set(float(height))
             
    # Common Xform
    UsdGeom.XformCommonAPI(prim_geom).SetTranslate(Gf.Vec3d(position[0], position[1], position[2]))
    
    # Color
    if color:
        prim_geom.GetDisplayColorAttr().Set([Gf.Vec3f(color[0], color[1], color[2])])
        
    # PHYSICS COLLISION (THE NUCLEAR OPTION)
    prim = prim_geom.GetPrim()
    
    # 1. Collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    collision_api.CreateCollisionEnabledAttr(True)
    
    # 2. Rigid Body API (Kinematic)
    # This forces PhysX to track the object even if it's static.
    rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb_api.CreateKinematicEnabledAttr(True)
    
    return prim

def create_scene():
    stage = omni.usd.get_context().get_stage()
    
    # Physics Scene
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    
    # --- LIGHTING (Crucial for Visibility) ---
    light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    light.CreateIntensityAttr(800.0) # Bright
    light.CreateAngleAttr(0.53) # Sun angle
    # Rotate light to look down-ish
    # XformOp not easy on Lux directly without Xform wrapper?
    # UsdLux inherits from UsdGeomXformable
    # Rotation -60 deg X, 30 deg Y
    UsdGeom.XformCommonAPI(light).SetRotate(Gf.Vec3f(-60, 30, 0))
    
    # --- TEST OBJECTS (Requests: 2 Cubes, 1 Cylinder, 1 Sphere) ---
    
    # 1. Cube 1 (Red Wall) - X=2.0
    create_primitive(stage, "Cube", "/World/Cube_Red", 
                     position=[2.0, 0.0, 0.5], 
                     scale=[0.5, 2.0, 1.0], 
                     color=[0.8, 0.1, 0.1])
                     
    # 2. Cube 2 (Green Box) - X=1.0, Y=-1.0
    create_primitive(stage, "Cube", "/World/Cube_Green", 
                     position=[1.0, -1.0, 0.25], 
                     scale=[0.5, 0.5, 0.5], 
                     color=[0.1, 0.8, 0.1])

    # 3. Sphere (Blue) - X=2.5, Y=1.0
    create_primitive(stage, "Sphere", "/World/Sphere_Blue",
                     position=[2.5, 1.0, 0.5],
                     radius=0.5,
                     color=[0.1, 0.1, 0.8])
                     
    # 4. Cylinder (Yellow) - X=1.5, Y=0.0
    create_primitive(stage, "Cylinder", "/World/Cylinder_Yellow",
                     position=[1.5, 0.0, 0.5],
                     radius=0.3,
                     height=1.0,
                     color=[0.8, 0.8, 0.1])
    
    # Save
    import os
    import time
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/test_env.usd"))
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

if __name__ == "__main__":
    try:
        print("[INFO] Generating Test Env (Cubes+Sphere)...")
        create_scene()
        print("[INFO] Generation Success.")
    except Exception as e:
        print(f"[FATAL] Generation Failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
