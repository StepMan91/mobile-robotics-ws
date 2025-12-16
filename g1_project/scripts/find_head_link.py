from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

USD_PATH = "c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"

print(f"Opening stage: {USD_PATH}")
stage = Usd.Stage.Open(USD_PATH)

print("-" * 50)
print("SEARCHING FOR HEAD/SENSOR LINKS:")
print("-" * 50)

found_head = False
for prim in stage.Traverse():
    name = prim.GetName()
    path = prim.GetPath().pathString
    
    # Check if it's a visual or body mesh
    if "head" in name.lower() or "face" in name.lower() or "camera" in name.lower() or "lidar" in name.lower():
        print(f"CANDIDATE: {name} -> {path}")
        found_head = True
    
    # Also print any rigid body that might be relevant
    if prim.IsA(UsdGeom.Imageable):
        # Just to see structure if we miss 'head'
        # print(f"DEBUG: {path}") 
        pass

if not found_head:
    print("WARNING: No explicit 'head' link found. Printing ALL children of root:")
    root = stage.GetPseudoRoot()
    for child in root.GetChildren():
        print(f"ROOT CHILD: {child.GetName()}")

print("-" * 50)
simulation_app.close()
