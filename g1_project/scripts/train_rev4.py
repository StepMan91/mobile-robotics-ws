
import argparse
import sys
import os
import torch

# Add paths manually
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

# RSL_RL
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import hydra
from omegaconf import DictConfig
from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_rev4_env_cfg import G1Rev4EnvCfg # REV4 CONFIG
from rsl_rl.runners import OnPolicyRunner # STANDARD PPO (No PER)

# Wrapper (Inline)
class RslRlVecEnvWrapper:
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
        return policy_obs, policy_obs, rew, dones, extras

    def get_observations(self):
        policy_obs = self.env.unwrapped.observation_manager.compute()["policy"]
        return policy_obs, policy_obs
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        policy_obs = obs_dict["policy"]
        return policy_obs, policy_obs

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args()

    # 1. Environment (Rev4)
    env_cfg = G1Rev4EnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0"
    
    print(f"[INFO] Creating Environment Rev4 with {args.num_envs} envs on {env_cfg.sim.device}")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 2. Wrapper
    vec_env = RslRlVecEnvWrapper(env)
    
    # 3. Log Dir
    log_dir = os.path.join(script_dir, "logs_rev4")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        
    # 4. PPO Config (Standard)
    ppo_config = {
        "seed": 42,
        "device": "cuda:0",
        "num_steps_per_env": 24,
        "max_iterations": 5000, # 5000 Epochs
        "save_interval": 100,
        "empirical_normalization": False,
        "policy": {
            "init_noise_std": 1.0,
            "actor_hidden_dims": [128, 64, 32], # Smaller, faster net for Rev4
            "critic_hidden_dims": [128, 64, 32],
            "activation": "elu",
        },
        "algorithm": {
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
    }

    # Reset before runner
    print("[INFO] Resetting Env...")
    vec_env.reset()
    
    print("[INFO] Starting Standard PPO Runner...")
    runner = OnPolicyRunner(vec_env, ppo_config, log_dir=log_dir, device="cuda:0")
    runner.learn(num_learning_iterations=5000, init_at_random_ep_len=True)
    
    print("[INFO] Training Complete.")
    simulation_app.close()

if __name__ == "__main__":
    main()
