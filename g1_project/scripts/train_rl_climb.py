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
# Headless for training
simulation_app = SimulationApp({"headless": True})

import torch
import hydra
from omegaconf import DictConfig

from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_climb_env_cfg import G1ClimbEnvCfg
from per_components import PrioritizedRunner, PrioritizedPPO
from tensordict import TensorDict

# Wrapper (Reused from play_rl_per_999.py)
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
            # print(f"[WARNING] {name} contains NaN/Inf! replacing with zeros.", flush=True)
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
    # 1. Create Environment
    env_cfg = G1ClimbEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # WRAP ENV FOR RSL-RL
    env = RslRlVecEnvWrapper(env)
    
    print("[DEBUG] Environment created and wrapped.", flush=True)
    
    # 2. Config for PPO/PER
    # Manually defining strict config for massive training
    alg_cfg = {
        "value_loss_coef": 1.0,
        "use_clipped_value_loss": True,
        "clip_param": 0.2,
        "entropy_coef": 0.01,
        "num_learning_epochs": 5,
        "num_mini_batches": 4, # 4096 / 4 = 1024 batch size
        "learning_rate": 1.0e-3,
        "schedule": "adaptive",
        "gamma": 0.99,
        "lam": 0.95,
        "desired_kl": 0.01,
        "max_grad_norm": 1.0,
    }
    
    # PER Components (Already in per_components.py)
    # Runner handles the storage and wrapping
    
    log_dir = os.path.join(os.getcwd(), "g1_project", "scripts", "logs_per_climb")
    
    # Reset Environment to ensure observations are valid
    print("[DEBUG] Resetting environment...", flush=True)
    env.reset()
    print("[DEBUG] Environment reset done.", flush=True)
    
    # Flattened Config for OnPolicyRunner
    train_cfg = {
        "seed": 42,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "num_steps_per_env": 24,
        # "max_iterations": 100, # VERIFICATION: 100 EPOCHS
        "max_iterations": 1000, # USER REQUEST: 1000 EPOCHS
        "save_interval": 50, # USER REQUEST: 50
        "experiment_name": "climb_per_rev2", # NEW EXPERIMENT FOR OVERHAUL
        "run_name": "run_001",
        "resume": False,
        "load_run": -1,
        "checkpoint": -1,
        "algorithm": {
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4, # 4096 / 4 = 1024 batch size
            "learning_rate": 3.0e-4, # REDUCED LR FOR STABILITY (Rev2)
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
        "policy": {
             "class_name": "ActorCritic", # Required by OnPolicyRunner
             "init_noise_std": 1.0,
             "noise_std_type": "log", # [FIX] Force positive std via log parameterization
             "actor_hidden_dims": [128, 64, 32],
             "critic_hidden_dims": [128, 64, 32],
             "activation": "elu",
        },
    }
    
    print("[DEBUG] Creating Runner...", flush=True)
    runner = PrioritizedRunner(
        env=env,
        train_cfg=train_cfg,
        log_dir=log_dir,
        device="cuda:0"
    )
    print("[DEBUG] Runner created. Starting Learning...", flush=True)
    
    # MANUAL RESUME (Disabled for Rev2 fresh start)
    # resume_path = os.path.join(log_dir, "model_500.pt")
    # if os.path.exists(resume_path):
    #      print(f"[INFO] Resuming training from: {resume_path}", flush=True)
    #      runner.load(resume_path)
    
    runner.learn(num_learning_iterations=train_cfg["max_iterations"], init_at_random_ep_len=True)
    print("[DEBUG] Learning finished.", flush=True)
    
    simulation_app.close()

if __name__ == "__main__":
    main()
