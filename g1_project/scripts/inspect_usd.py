from isaacsim import SimulationApp

config = {"headless": True}
simulation_app = SimulationApp(config)

import omni.usd
from pxr import Usd, UsdGeom

usd_path = r"c:/Users/basti/source/repos/mobile-robotics-ws/g1_project/assets/stairs_env.usd"
print(f"Opening stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
stage = omni.usd.get_context().get_stage()

with open(r"c:\Users\basti\source\repos\mobile-robotics-ws\usd_prims.txt", "w") as f:
    print("Traversing stage...", file=f)
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            print(f"[MESH] {prim.GetPath()}", file=f)
        else:
            print(f"[PRIM] {prim.GetPath()} ({prim.GetTypeName()})", file=f)

simulation_app.close()
