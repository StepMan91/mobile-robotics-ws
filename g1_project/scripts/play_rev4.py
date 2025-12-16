
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
simulation_app = SimulationApp({"headless": False}) # VISUALIZATION MODE

import hydra
from omegaconf import DictConfig
from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_rev4_env_cfg import G1Rev4EnvCfg
from rsl_rl.runners import OnPolicyRunner
from tensordict import TensorDict

# Wrapper (Same as Train)
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
        
    def _to_tensordict(self, obs_dict):
        return TensorDict(obs_dict, batch_size=self.num_envs, device=self.device)

    def step(self, actions):
        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        dones = terminated | truncated
        return self._to_tensordict(obs_dict), rew, dones, extras

    def get_observations(self):
        obs_dict = self.env.unwrapped.observation_manager.compute()
        return self._to_tensordict(obs_dict)
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        return self._to_tensordict(obs_dict), {"info": {}}

    def __getattr__(self, name):
        return getattr(self.env, name)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=1) # Single robot for view
    parser.add_argument("--checkpoint", type=str, default=None) # Path to model
    args = parser.parse_args()

    # 1. Environment
    env_cfg = G1Rev4EnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0"
    
    print(f"[INFO] Creating Play Env (Rev4) with {args.num_envs} envs...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 2. Wrapper
    vec_env = RslRlVecEnvWrapper(env)
    
    # 3. PPO Config (Minimal for Loading)
    log_dir = os.path.join(script_dir, "logs_rev4")
    # Finding latest model if not specified
    if args.checkpoint is None:
        # Simple find latest
        files = [f for f in os.listdir(log_dir) if f.endswith(".pt")]
        if not files:
            print("[ERROR] No models found in logs_rev4!")
            return
        files.sort(key=lambda x: os.path.getmtime(os.path.join(log_dir, x)))
        latest_model = files[-1]
        args.checkpoint = os.path.join(log_dir, latest_model)
    
    print(f"[INFO] Loading Checkpoint: {args.checkpoint}")
    
    # Structure must match training
    ppo_config = {
        "seed": 42,
        "device": "cuda:0",
        "num_steps_per_env": 24,
        "max_iterations": 5000, 
        "save_interval": 100,
        "empirical_normalization": False,
        "policy": {
            "class_name": "ActorCritic",
            "init_noise_std": 1.0,
            "actor_hidden_dims": [128, 64, 32],
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
        "obs_groups": {"policy": ["policy"]},
    }

    runner = OnPolicyRunner(vec_env, ppo_config, log_dir=log_dir, device="cuda:0")
    runner.load(args.checkpoint)
    
    # 4. Inference
    policy = runner.get_inference_policy(device="cuda:0")
    
    print("[INFO] Starting Play Loop. Press Ctrl+C to stop.")
    
    obs, _ = vec_env.reset()
    
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, rewards, dones, extras = vec_env.step(actions)
            
    simulation_app.close()

if __name__ == "__main__":
    main()
