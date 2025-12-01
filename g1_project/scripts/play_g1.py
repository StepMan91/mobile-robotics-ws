import argparse
import sys
import os

# Add the source directory to sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add Isaac Lab source paths
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
sys.path.append(os.path.join(isaac_lab_path, "isaaclab"))

from isaacsim import SimulationApp

# Parse arguments
parser = argparse.ArgumentParser(description="Play G1 Locomotion")
parser.add_argument("--task", type=str, default="Isaac-Locomotion-G1-v0", help="Task name")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
args_cli = parser.parse_args()

# Launch the app with GUI
config = {"headless": False}
simulation_app = SimulationApp(config)

import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedRLEnv
import g1_locomotion
from g1_locomotion.g1_env_cfg import G1LocomotionEnvCfg

def main():
    # Instantiate configuration
    env_cfg = G1LocomotionEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    
    # Create the environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    
    print(f"[INFO] Environment created: {env.unwrapped.cfg.scene.num_envs} environments")
    
    # Reset the environment
    obs, _ = env.reset()
    
    # Simulation loop
    print("[INFO] Starting simulation loop...")
    while simulation_app.is_running():
        # Zero actions for now (standing still / falling)
        # actions = torch.zeros((env.unwrapped.num_envs, env.unwrapped.action_space.shape[1]), device=env.unwrapped.device)
        
        # Random actions to see movement
        actions = torch.rand((env.unwrapped.num_envs, env.unwrapped.action_space.shape[1]), device=env.unwrapped.device) * 2 - 1
        
        # Step the environment
        obs, rew, terminated, truncated, info = env.step(actions)
        
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
