# Import Simulation App
from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": True}
simulation_app = SimulationApp(CONFIG)

import omni.usd
import numpy as np
import math
import os
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid, VisualCylinder
from pxr import Gf, UsdLux, UsdPhysics

# Defaults from Pantin/climb_stairs.py
STEP_HEIGHT = 0.15
STEP_DEPTH = 0.25
WIDTH = 1.0

def apply_collision_rigid(prim_path, world):
    stage = world.stage
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid(): return
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)

def create_industrial_stairs(world, position, num_steps=15, step_height=STEP_HEIGHT, step_depth=STEP_DEPTH, width=WIDTH):
    base_pos = np.array(position)
    
    # Stairs
    for i in range(num_steps):
        x_offset = i * step_depth
        z_offset = i * step_height + (step_height / 2.0)
        pos = base_pos + np.array([x_offset, 0, z_offset])
        prim_path = f"/World/Environment/Stairs/Step_{i}"
        world.scene.add(
            VisualCuboid(
                prim_path=prim_path, 
                name=f"step_{i}", 
                position=pos, 
                scale=np.array([step_depth, width, step_height]), 
                color=np.array([0.3, 0.3, 0.35])
            )
        )
        apply_collision_rigid(prim_path, world)

    # Catwalk
    catwalk_depth = 2.0
    catwalk_pos = base_pos + np.array([(num_steps * step_depth) + (catwalk_depth / 2.0) - (step_depth), 0, (num_steps - 1) * step_height + (step_height / 2.0)])
    cw_path = "/World/Environment/Stairs/Catwalk"
    world.scene.add(
        VisualCuboid(
            prim_path=cw_path, 
            name="catwalk", 
            position=catwalk_pos, 
            scale=np.array([catwalk_depth, width, step_height]), 
            color=np.array([0.25, 0.25, 0.3])
        )
    )
    apply_collision_rigid(cw_path, world)

    # Handrails
    rail_height = 0.9
    total_run = (num_steps - 1) * step_depth
    total_rise = (num_steps - 1) * step_height
    diag_len = math.sqrt(total_run**2 + total_rise**2)
    angle_rad = math.atan2(total_rise, total_run)
    center_x = (total_run / 2.0)
    center_z = (total_rise / 2.0) + rail_height + step_height
    rail_offsets_y = [width/2.0, -width/2.0]
    pitch_deg = 90 - math.degrees(angle_rad)
    
    for idx, y_off in enumerate(rail_offsets_y):
        rail_pos = base_pos + np.array([center_x, y_off, center_z])
        # Orientation for Y-rotation
        rad = math.radians(pitch_deg)
        orient = np.array([math.cos(rad/2), 0, math.sin(rad/2), 0])
        
        world.scene.add(
            VisualCylinder(
                prim_path=f"/World/Environment/Stairs/Rail_Diag_{idx}", 
                name=f"rail_diag_{idx}", 
                position=rail_pos, 
                scale=np.array([0.025, 0.025, diag_len + 0.5]), 
                color=np.array([0.8, 0.8, 0.2]), 
                orientation=orient
            )
        )
        
        post_indices = [0, num_steps // 2, num_steps - 1]
        for p_idx in post_indices:
             px = p_idx * step_depth
             pz = p_idx * step_height + step_height
             post_pos = base_pos + np.array([px, y_off, pz + rail_height/2.0])
             world.scene.add(
                VisualCylinder(
                    prim_path=f"/World/Environment/Stairs/Post_{idx}_{p_idx}", 
                    name=f"post_{idx}_{p_idx}", 
                    position=post_pos, 
                    scale=np.array([0.02, 0.02, rail_height]), 
                    color=np.array([0.2, 0.2, 0.2])
                )
             )

def create_lighting_array(stage, start_pos, count=8, spacing=3.0, height=3.0):
    for i in range(count):
        x = start_pos[0] + i * spacing
        pos = Gf.Vec3f(x, start_pos[1], start_pos[2] + height)
        light_path = f"/World/Lights/Light_{i}"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateIntensityAttr(80000.0) 
        light.CreateRadiusAttr(0.15)
        light.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0)) 
        light.AddTranslateOp().Set(pos)

def create_floor_markings(world, start_pos, end_pos):
    # Concrete Floor
    floor_path = "/World/Environment/ConcreteFloor"
    world.scene.add(
        VisualCuboid(
            prim_path=floor_path,
            name="concrete_floor",
            position=np.array([5.0, 0.0, -0.05]), 
            scale=np.array([20.0, 10.0, 0.1]),
            color=np.array([0.2, 0.2, 0.2])
        )
    )
    apply_collision_rigid(floor_path, world)
    
    # Blue Path
    dist = end_pos[0] - start_pos[0]
    center_x = start_pos[0] + dist / 2.0
    
    path_path = "/World/Environment/BluePath"
    world.scene.add(
        VisualCuboid(
            prim_path=path_path,
            name="blue_path",
            position=np.array([center_x, 0.0, 0.005]), 
            scale=np.array([dist, 0.6, 0.01]), 
            color=np.array([0.0, 0.2, 0.8])
        )
    )

def create_scene():
    world = World()
    stage = omni.usd.get_context().get_stage()

    # Ground Plane (keep default for infinity physics, but hide it usually?)
    world.scene.add_default_ground_plane()
    
    # 1. Lights
    create_lighting_array(stage, start_pos=(-2, 0, 0), count=8, spacing=3.0)
    
    # 2. Floor & Markings
    create_floor_markings(world, start_pos=[0,0,0], end_pos=[3,0,0])
    
    # 3. Stairs (at X=3.0)
    S_NUM = 15; S_H = 0.15; S_D = 0.25
    create_industrial_stairs(world, position=[3.0, 0.0, 0.0], num_steps=S_NUM, step_height=S_H, step_depth=S_D)
    
    # Set Default Prim
    root_prim = stage.GetPrimAtPath("/World")
    if root_prim.IsValid():
        stage.SetDefaultPrim(root_prim)
    
    # Save
    save_path = os.path.abspath(os.path.join(os.getcwd(), "g1_project/assets/climb_world.usd"))
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    print(f"Saving world to {save_path}...")
    omni.usd.get_context().save_as_stage(save_path)
    print("Done.")

import traceback

if __name__ == "__main__":
    try:
        print("[INFO] Starting World Export...")
        create_scene()
        print("[INFO] Export Finished Successfully.")
    except Exception:
        print("[FATAL] Export Failed:")
        traceback.print_exc()
    finally:
        simulation_app.close()
