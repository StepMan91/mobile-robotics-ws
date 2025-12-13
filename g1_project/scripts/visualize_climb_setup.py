import os
import sys

# Add source path
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
core_path = os.path.join(isaac_lab_path, "isaaclab")
if core_path not in sys.path:
    sys.path.append(core_path)

ext_path = os.path.join(isaac_lab_path, "extensions")
if ext_path not in sys.path:
    sys.path.append(ext_path)

# Add Local rsl_rl repo
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.insert(0, rsl_rl_path)

# Launch Isaac Sim
from isaacsim import SimulationApp
# VISUAL MODE
simulation_app = SimulationApp({"headless": False})

import torch
from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_climb_env_cfg import G1ClimbEnvCfg

def main():
    print("[INFO] Setting up Visualization Environment...")
    
    # Force 1 Env for Visual
    env_cfg = G1ClimbEnvCfg()
    env_cfg.scene.num_envs = 1
    # Disable randomization for consistent look if desired
    # env_cfg.events.reset_base.params["pose_range"] = {"x": (1.0, 1.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("[INFO] Environment Ready. Starting Loop...")
    
    obs, _ = env.reset()
    
    while simulation_app.is_running():
        # Do nothing (Zero actions)
        # Just to visualize the robot standing/falling in the scene
        zero_actions = torch.zeros(env.action_space.shape, device=env.device)
        
        obs, rew, terminated, truncated, info = env.step(zero_actions)
        
        if terminated.any() or truncated.any():
            env.reset()
            
    simulation_app.close()

if __name__ == "__main__":
    main()
