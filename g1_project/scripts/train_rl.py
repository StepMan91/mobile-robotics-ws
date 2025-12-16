
import argparse
import sys
import os
import torch

# Add source path
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path (Parent of isaaclab package)
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
# As per train_g1.py:
core_path = os.path.join(isaac_lab_path, "isaaclab")
if core_path not in sys.path:
    sys.path.append(core_path)
    print(f"[INFO] Appended {core_path} to sys.path")

ext_path = os.path.join(isaac_lab_path, "extensions")
if ext_path not in sys.path:
    sys.path.append(ext_path)
    print(f"[INFO] Appended {ext_path} to sys.path")

# Add Local rsl_rl repo (Fix for import error)
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)
    print(f"[INFO] Appended {rsl_rl_path} to sys.path")

# Launch Isaac Sim
from isaacsim import SimulationApp
config = {"headless": True}
simulation_app = SimulationApp(config)

# Imports after Sim Start
# Imports after Sim Start
import traceback
try:
    import gymnasium as gym
    print("[INFO] Imported gymnasium")
    from isaaclab.envs import ManagerBasedRLEnv
    print("[INFO] Imported ManagerBasedRLEnv")
    # Wrapper to make IsaacLab Env compatible with RSL-RL
    # from isaaclab.envs import DirectMARLEnv, DirectRLEnv # Check wrappers
    # Usually we wrap the gym env.
    from rsl_rl.runners import OnPolicyRunner
    print("[INFO] Imported OnPolicyRunner")

    # Import Config
    import g1_locomotion
    from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg
    print("[INFO] Imported G1StairsEnvCfg")
except Exception as e:
    print(f"[ERROR] Import failed: {e}")
    traceback.print_exc()
    sys.exit(1)

class RslRlVecEnvWrapper:
    """Wrapper to make IsaacLab Gym Env compatible with RSL-RL."""
    def __init__(self, env):
        self.env = env
        self.num_envs = env.unwrapped.num_envs
        # Check action/obs spaces
        if hasattr(self.env.unwrapped, "num_actions"):
            self.num_actions = self.env.unwrapped.num_actions
        else:
            self.num_actions = self.env.unwrapped.action_space.shape[1]
            
        if hasattr(self.env.unwrapped, "num_observations"):
            self.num_obs = self.env.unwrapped.num_observations
        else:
            # Try to get from observation space
            # obs space is Dict.
            self.num_obs = self.env.unwrapped.observation_space['policy'].shape[1]
            
        self.num_privileged_obs = None 
        self.device = env.unwrapped.device
        
    def step(self, actions):
        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        dones = terminated | truncated
        # Returns: obs, privileged_obs, rewards, dones, infos
        policy_obs = obs_dict["policy"]
        return policy_obs, None, rew, dones, extras

    def get_observations(self):
        # Recompute observations
        # This accesses the internal manager
        return self.env.unwrapped.observation_manager.compute()["policy"]
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        return obs_dict["policy"], None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=100)
    args = parser.parse_args()

    # Config
    env_cfg = G1StairsEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0" # Force GPU
    
    # Create Env
    # Note: ManagerBasedRLEnv expects env_cfg
    env = gym.make("Isaac-Locomotion-G1-v0", cfg=env_cfg)
    
    # Wrap
    print("[INFO] Wrapping environment with local RslRlVecEnvWrapper...")
    vec_env = RslRlVecEnvWrapper(env)

    print("[INFO] Setting up PPO Runner...")
    
    # DEBUG: Print Stage Paths
    from omni.isaac.core.utils.stage import get_current_stage
    stage = get_current_stage()
    if stage:
        print("[DEBUG] Stage Traversal (/World/ground):")
        ground = stage.GetPrimAtPath("/World/ground")
        if ground.IsValid():
            for p in ground.GetChildren():
                print(f"  {p.GetPath()}")
                for child in p.GetChildren():
                    print(f"    {child.GetPath()}")
                    for grand in child.GetChildren():
                        print(f"      {grand.GetPath()}") # Depth 3
        else:
            print("[DEBUG] /World/ground not found!")
    
    # RSL-RL Config
    ppo_config = {
        "seed": 42,
        "runner": {
            "policy_class_name": "ActorCritic",
            "algorithm_class_name": "PPO",
            "num_steps_per_env": 24,
            "max_iterations": 100, # Short run for verification
            "save_interval": 25,
            "experiment_name": "g1_stairs",
            "run_name": "v1",
            "resume": False,
            "load_run": -1,
            "checkpoint": -1,
            "resume_path": None,
        },
        "algorithm": {
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "policy": {
            "init_noise_std": 1.0,
            "actor_hidden_dims": [128, 64, 32], # Smaller net for speed
            "critic_hidden_dims": [128, 64, 32],
            "activation": "elu", 
        }
    }
    
    log_dir = os.path.abspath(os.path.join(script_dir, "logs"))
    print(f"[INFO] Logging to: {log_dir}")
    
    runner = OnPolicyRunner(vec_env, ppo_config, log_dir=log_dir, device=env_cfg.sim.device)
    
    print("[INFO] Starting Training...")
    runner.learn(num_learning_iterations=100, init_at_random_ep_len=True)
    
    print("[INFO] Training Finished.")
    simulation_app.close()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[FATAL ERROR] Main Crashed: {e}")
        traceback.print_exc()
        sys.exit(1)
