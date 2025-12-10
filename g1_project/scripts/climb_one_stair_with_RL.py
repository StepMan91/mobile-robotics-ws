
import argparse
import sys
import os
import torch
import gymnasium as gym

# Add source path
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
if isaac_lab_path not in sys.path:
    sys.path.append(isaac_lab_path)
ext_path = os.path.join(isaac_lab_path, "extensions")
if ext_path not in sys.path:
    sys.path.append(ext_path)

# Add Local rsl_rl repo
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)

# Launch Isaac Sim
from isaacsim import SimulationApp
config = {"headless": False} # Visual mode
simulation_app = SimulationApp(config)

# Imports after Sim Start
from isaaclab.envs import ManagerBasedRLEnv
from rsl_rl.modules import ActorCritic
from rsl_rl.env import VecEnv

import g1_locomotion
from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg

class RslRlVecEnvWrapper:
    """Wrapper to make IsaacLab Gym Env compatible with RSL-RL."""
    def __init__(self, env):
        self.env = env
        self.num_envs = env.unwrapped.num_envs
        if hasattr(self.env.unwrapped, "num_actions"):
            self.num_actions = self.env.unwrapped.num_actions
        else:
            self.num_actions = self.env.unwrapped.action_space.shape[1]
        
        if hasattr(self.env.unwrapped, "num_observations"):
            self.num_obs = self.env.unwrapped.num_observations
        else:
            self.num_obs = self.env.unwrapped.observation_space['policy'].shape[1]
            
        self.num_privileged_obs = None 
        self.device = env.unwrapped.device
        
    def step(self, actions):
        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        dones = terminated | truncated
        policy_obs = obs_dict["policy"]
        return policy_obs, None, rew, dones, extras

    def get_observations(self):
        return self.env.unwrapped.observation_manager.compute()["policy"]
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        return obs_dict["policy"], None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str, default=None, help="Path to model .pt file")
    args = parser.parse_args()

    # Config
    env_cfg = G1StairsEnvCfg()
    env_cfg.scene.num_envs = 1 # Single robot
    env_cfg.sim.device = "cuda:0"
    
    # Create Env
    env = gym.make("Isaac-Locomotion-G1-v0", cfg=env_cfg)
    vec_env = RslRlVecEnvWrapper(env)

    # Load Policy
    # Assuming standard rsl_rl ActorCritic structure
    # Must match training config!
    actor_hidden_dims = [128, 64, 32]
    critic_hidden_dims = [128, 64, 32]
    activation = "elu"
    
    print(f"[INFO] Obs: {vec_env.num_obs}, Actions: {vec_env.num_actions}")
    
    policy = ActorCritic(
        vec_env.num_obs,
        vec_env.num_privileged_obs,
        vec_env.num_actions,
        actor_hidden_dims=actor_hidden_dims,
        critic_hidden_dims=critic_hidden_dims,
        activation=activation,
    ).to(vec_env.device)
    
    # Checkpoint Path
    if args.checkpoint:
        ckpt_path = args.checkpoint
    else:
        # Default to latest in logs
        log_root = os.path.join(script_dir, "logs/g1_stairs")
        # Find latest run
        if os.path.exists(log_root):
             runs = sorted(os.listdir(log_root))
             if runs:
                 last_run = runs[-1]
                 # Look for model_last.pt
                 p = os.path.join(log_root, last_run, "model_last.pt")
                 if os.path.exists(p):
                     ckpt_path = p
                 else:
                     # fallback
                     model_files = [f for f in os.listdir(os.path.join(log_root, last_run)) if f.startswith("model_")]
                     if model_files:
                         ckpt_path = os.path.join(log_root, last_run, model_files[-1])
    
    if ckpt_path and os.path.exists(ckpt_path):
        print(f"[INFO] Loading checkpoint: {ckpt_path}")
        loaded_dict = torch.load(ckpt_path)
        policy.load_state_dict(loaded_dict['model_state_dict']) # Usually saved as full dict
    else:
        print("[WARN] No checkpoint found. Running with random weights.")
    
    policy.eval()
    
    # Simulation Loop
    obs, _ = vec_env.reset()
    
    print("[INFO] Starting Inference Loop...")
    while simulation_app.is_running():
        with torch.no_grad():
            actions = policy.act_inference(obs)
            
        obs, _, _, _, _ = vec_env.step(actions)
        
    simulation_app.close()

if __name__ == "__main__":
    main()
