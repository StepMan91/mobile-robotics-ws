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

def main():
    # 1. Create Environment
    env_cfg = G1ClimbEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)

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
    
    runner = PrioritizedRunner(
        env=env,
        train_cfg={
            "runner": {
                "algorithm": alg_cfg,
                "policy": {
                     "actor_hidden_dims": [128, 64, 32],
                     "critic_hidden_dims": [128, 64, 32],
                     "activation": "elu",
                },
                # "max_iterations": 10, # DRY RUN: 10 Iterations
                "max_iterations": 2000, # USER REQUEST: 2000 EPOCHS
                "save_interval": 50,
                "experiment_name": "climb_per",
                "run_name": "run_001",
                "resume": False,
                "load_run": -1,
                "checkpoint": -1,
            }
        },
        log_dir=log_dir,
        device="cuda:0"
    )

    print(f"[INFO] Starting Training for {runner.max_iterations} iterations...")
    runner.learn(num_learning_iterations=runner.max_iterations, init_at_random_ep_len=True)
    
    simulation_app.close()

if __name__ == "__main__":
    main()
