# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

def create_primitive(stage, prim_type, path, position, scale=None, radius=None, color=None):
    if prim_type == "Cube":
        prim_geom = UsdGeom.Cube.Define(stage, path)
        if scale:
             s = Gf.Vec3d(scale[0]/2.0, scale[1]/2.0, scale[2]/2.0)
             UsdGeom.XformCommonAPI(prim_geom).SetScale(s)
             
    elif prim_type == "Sphere":
        prim_geom = UsdGeom.Sphere.Define(stage, path)
        if radius:
             prim_geom.GetRadiusAttr().Set(float(radius))
             
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
    
    # 3. Mass API (Mass = 0 or infinite for static? Kinematic ignores mass, but let's be safe)
    # Actually, RigidBodyAPI implies it.
    
    return prim

def create_scene():
    stage = omni.usd.get_context().get_stage()
    
    # Physics Scene
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    
    # Ground Plane
    plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    
    # --- TEST OBJECTS ---
    # 1. Cube 1 (Large - The Wall)
    # Position X=2.0 (In front of robot)
    create_primitive(stage, "Cube", "/World/Cube_Wall", 
                     position=[2.0, 0.0, 0.5], # Z=0.5 -> Center at 0.5 (height 1m)
                     scale=[0.5, 2.0, 1.0], # Thin wall
                     color=[0.8, 0.2, 0.2]) # Red
                     
    # 2. Cube 2 (Small - Step)
    # Position X=1.0 (Closer)
    create_primitive(stage, "Cube", "/World/Cube_Step", 
                     position=[1.0, 0.5, 0.25], 
                     scale=[0.5, 0.5, 0.5], 
                     color=[0.2, 0.8, 0.2]) # Green

    # 3. Sphere (Obstacle)
    # Position X=3.0
    create_primitive(stage, "Sphere", "/World/Sphere_Obs",
                     position=[3.0, -0.5, 0.5],
                     radius=0.5,
                     color=[0.2, 0.2, 0.8]) # Blue
    
    # Save
    import os
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/test_env.usd"))
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    print(f"Saving stage to {save_path}...")
    omni.usd.get_context().save_as_stage(save_path)
    print("Done.")

if __name__ == "__main__":
    try:
        print("[INFO] Generating Test Env (Cubes+Sphere)...")
        create_scene()
        print("[INFO] Generation Success.")
    except Exception as e:
        print(f"[FATAL] Generation Failed: {e}")
    finally:
        simulation_app.close()
