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
from isaaclab_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper 
from rsl_rl.runners import OnPolicyRunner
from per_components import PrioritizedRunner # Using PER

def main():
    # Configure Environment
    env_cfg = G1Rev3bEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device

    print(f"[INFO] Training Rev3b with {env_cfg.scene.num_envs} environments.")
    print(f"[INFO] Stiffness: 200.0 (per config)")

    # Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Wrap for RSL-RL
    vec_env = RslRlVecEnvWrapper(env)
    
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
