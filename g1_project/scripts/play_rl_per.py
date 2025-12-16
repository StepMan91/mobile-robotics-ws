
# play_rl_per.py
# Visualization script for PER trained policy

import argparse
import sys
import os
import torch
import traceback
import signal

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

# Add Local rsl_rl repo (Fix for import error)
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.insert(0, rsl_rl_path)

# Launch Isaac Sim (Headless=False for visual)
from isaacsim import SimulationApp
config = {"headless": False} # VISUALIZATION MODE
simulation_app = SimulationApp(config)

# Imports after Sim Start
try:
    import gymnasium as gym
    from isaaclab.envs import ManagerBasedRLEnv
    from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg
    from tensordict import TensorDict
    
    # Import PER Components
    if script_dir not in sys.path:
        sys.path.append(script_dir)
    from per_components import PrioritizedRunner
    
except Exception as e:
    print(f"[ERROR] Import failed: {e}")
    traceback.print_exc()
    sys.exit(1)

# Wrapper (Reused from train_rl.py)
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
        
    def step(self, actions):
        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        dones = terminated | truncated
        # Returns: obs, privileged_obs, rewards, dones, infos
        policy_obs = obs_dict["policy"]
        return TensorDict({"policy": policy_obs}, batch_size=[self.num_envs]), rew, dones, extras

    def get_observations(self):
        # Recompute observations
        return TensorDict({"policy": self.env.unwrapped.observation_manager.compute()["policy"]}, batch_size=[self.num_envs])
        
    def reset(self):
        obs_dict, _ = self.env.reset()
        return TensorDict({"policy": obs_dict["policy"]}, batch_size=[self.num_envs]), None

    def __getattr__(self, name):
        return getattr(self.env.unwrapped, name)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=5) # Low num envs for visual
    parser.add_argument("--load_run", type=str, required=True, help="Name of run folder (e.g. v1_per)")
    parser.add_argument("--checkpoint", type=str, default="model_350.pt")
    args = parser.parse_args()

    # Config
    env_cfg = G1StairsEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0" # Force GPU
    
    # Create Env
    env = gym.make("Isaac-Locomotion-G1-v0", cfg=env_cfg)
    
    # Wrap
    vec_env = RslRlVecEnvWrapper(env)

    # RSL-RL Config (Dummy, mostly for structure)
    ppo_config = {
        "seed": 42,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "num_steps_per_env": 24,
        "max_iterations": 500,
        "save_interval": 50,
        "experiment_name": "g1_stairs_per",
        "run_name": args.load_run, # Load from this run
        "resume": True,
        "load_run": args.load_run,
        "checkpoint": -1, # We load manually often, or use runner logic
        "resume_path": None,
        "algorithm": { "class_name": "PPO", "value_loss_coef": 1.0, "use_clipped_value_loss": True, "clip_param": 0.2, "entropy_coef": 0.01, "num_learning_epochs": 5, "num_mini_batches": 4, "learning_rate": 1e-3, "schedule": "adaptive", "gamma": 0.99, "lam": 0.95, "desired_kl": 0.01, "max_grad_norm": 1.0 },
        "policy": { "class_name": "ActorCritic", "init_noise_std": 1.0, "actor_hidden_dims": [128, 64, 32], "critic_hidden_dims": [128, 64, 32], "activation": "elu" }
    }
    
    log_dir = os.path.abspath(os.path.join(script_dir, "logs_per"))
    
    # Reset Environment
    vec_env.reset()

    # Use PrioritizedRunner
    runner = PrioritizedRunner(vec_env, ppo_config, log_dir=log_dir, device=env_cfg.sim.device)
    
    # Load model
    # runner.load(resume_path) # Need to construct path
    resume_path = os.path.join(log_dir, "g1_stairs_per", args.load_run, "nn", args.checkpoint)
    print(f"[INFO] Loading model from: {resume_path}")
    runner.load(resume_path)
    
    policy = runner.alg.actor_critic
    policy.eval()
    
    print("[INFO] Starting Playback...")
    
    obs, _ = vec_env.reset()
    
    # Play loop
    while simulation_app.is_running():
        with torch.no_grad():
            # Get Action
            # obs is TensorDict
            actions = policy.act_inference(obs)
            
            # Step
            obs, rew, dones, extras = vec_env.step(actions)
            
    print("[INFO] Playback Finished.")
    simulation_app.close()

if __name__ == "__main__":
    main()
