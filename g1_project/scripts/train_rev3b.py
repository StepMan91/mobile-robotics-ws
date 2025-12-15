import argparse
import sys
import os
import torch

# Add paths manually (Robustness against Launcher environment issues)
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
core_path = os.path.join(isaac_lab_path, "isaaclab")
if core_path not in sys.path:
    sys.path.append(core_path)

# Extensions
ext_path = os.path.join(isaac_lab_path, "extensions")
if ext_path not in sys.path:
    sys.path.append(ext_path)

# RSL_RL
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)

# 1. Parse Args (Standard argparse)
parser = argparse.ArgumentParser(description="Train G1 Climbing (Riv3b) with RSL-RL.")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of environments.")
parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode.")
parser.add_argument("--device", type=str, default="cuda:0", help="Device to use.")
args = parser.parse_args()

# 2. Launch Isaac Sim (Directly)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# 3. Imports after App Launch
from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_rev3b_env_cfg import G1Rev3bEnvCfg # NEW CONFIG
# from isaaclab_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper # FAILED
from rsl_rl.runners import OnPolicyRunner
from per_components import PrioritizedRunner # Using PER
from tensordict import TensorDict

# Wrapper (Reused from play_rl_per_999.py / train_rl_climb.py)
class RslRlVecEnvWrapper:
    """Wrapper to make IsaacLab Gym Env compatible with RSL-RL."""
    def __init__(self, env):
        self.env = env
        self.cfg = env.unwrapped.cfg # Expose config for Logger
        self.num_envs = env.unwrapped.num_envs
        # Check action/obs spaces
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
        
    def _sanitize(self, tensor, name="Observation"):
        if torch.isnan(tensor).any() or torch.isinf(tensor).any():
            return torch.nan_to_num(tensor, nan=0.0, posinf=0.0, neginf=0.0)
        return tensor

    def step(self, actions):
        # Sanitize actions before sending to env
        actions = self._sanitize(actions, "Actions")
        
        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        dones = terminated | truncated
        # Returns: obs, privileged_obs, rewards, dones, infos
        policy_obs = obs_dict["policy"]
        
        # Sanitize outputs
        policy_obs = self._sanitize(policy_obs, "PolicyObs")
        rew = self._sanitize(rew, "Rewards")
        
        return TensorDict({"policy": policy_obs}, batch_size=[self.num_envs]), rew, dones, extras

    def get_observations(self):
        # Recompute observations
        obs = self.env.unwrapped.observation_manager.compute()["policy"]
        return TensorDict({"policy": self._sanitize(obs, "GetObs")}, batch_size=[self.num_envs])
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        obs = obs_dict["policy"]
        return TensorDict({"policy": self._sanitize(obs, "ResetObs")}, batch_size=[self.num_envs]), None

    def __getattr__(self, name):
        return getattr(self.env.unwrapped, name)

def main():
    print("[DEBUG] ENTERING MAIN", flush=True)
    # Configure Environment
    try:
        print("[DEBUG] Creating G1Rev3bEnvCfg...", flush=True)
        env_cfg = G1Rev3bEnvCfg()
        env_cfg.scene.num_envs = args.num_envs
        env_cfg.sim.device = args.device
        print("[DEBUG] G1Rev3bEnvCfg Created.", flush=True)
    except Exception as e:
        print(f"[ERROR] Logic Error in Config: {e}", flush=True)
        return

    print(f"[INFO] Training Rev3b with {env_cfg.scene.num_envs} environments.")
    print(f"[INFO] Stiffness: 200.0 (per config)")

    # Create Environment
    try:
        print("[DEBUG] Creating ManagerBasedRLEnv...", flush=True)
        env = ManagerBasedRLEnv(cfg=env_cfg)
        print("[DEBUG] ManagerBasedRLEnv Created.", flush=True)
    except Exception as e:
         print(f"[ERROR] Failed to create ManagerBasedRLEnv: {e}", flush=True)
         import traceback
         traceback.print_exc()
         return
    
    # Wrap for RSL-RL
    print("[DEBUG] Wrapping Environment...", flush=True)
    vec_env = RslRlVecEnvWrapper(env)
    print("[DEBUG] Environment Wrapped.", flush=True)
    
    # Configure RSL-RL (PPO + PER)
    # Adding Normalization!
    ppo_config = {
        "seed": 42,
        "device": env_cfg.sim.device,
        "num_steps_per_env": 24,
        "max_iterations": 3000, # 3000 Iterations (~72M steps)
        "save_interval": 100,   # Save often
        "experiment_name": "g1_climb_rev3b",
        "run_name": "run_001",
        "obs_groups": {
            "actor": ["policy"], 
            "critic": ["policy"]
        },
        "algorithm": {
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4, 
            "learning_rate": 1.0e-3, # Faster learning start
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
        "policy": {
            "class_name": "ActorCritic",
            "init_noise_std": 1.0,
            "actor_hidden_dims": [256, 128, 64], # Slightly deeper
            "critic_hidden_dims": [256, 128, 64],
            "activation": "elu",
            # CRITICAL: ENABLE NORMALIZATION
            "actor_obs_normalization": True,
            "critic_obs_normalization": True,
        }
    }

    # Log Directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "logs_rev3b")
    
    # Create Runner
    print(f"[INFO] Logging to: {log_dir}")
    runner = PrioritizedRunner(vec_env, ppo_config, log_dir=log_dir, device=env_cfg.sim.device)

    # Start Training
    runner.learn(num_learning_iterations=ppo_config["max_iterations"], init_at_random_ep_len=True)
    
    # Close
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
