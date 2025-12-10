
import argparse
import sys
import os
import torch

# Add source path
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path (if needed)
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
sys.path.append(os.path.join(isaac_lab_path, "isaaclab"))

# Launch Isaac Sim
from isaacsim import SimulationApp
config = {"headless": True}
simulation_app = SimulationApp(config)

# Imports after Sim Start
import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnv
# Wrapper to make IsaacLab Env compatible with RSL-RL
from isaaclab.envs import DirectMARLEnv, DirectRLEnv # Check wrappers
# Usually we wrap the gym env.
from rsl_rl.runners import OnPolicyRunner

# Import Config
import g1_locomotion
from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg

class RslRlVecEnvWrapper:
    """Wrapper to make IsaacLab Gym Env compatible with RSL-RL."""
    def __init__(self, env):
        self.env = env
        self.num_envs = env.unwrapped.num_envs
        self.num_obs = env.unwrapped.observation_space['policy'].shape[1]
        self.num_actions = env.unwrapped.action_space.shape[1]
        # RSL-RL expects num_privileged_obs
        self.num_privileged_obs = None # Not used here
        self.device = env.unwrapped.device
        
    def step(self, actions):
        obs, rew, terminated, truncated, infos = self.env.step(actions)
        dones = terminated | truncated
        # Returns: obs, privileged_obs, rewards, dones, infos
        # obs dictionary? rsl_rl expects tensor?
        # IsaacLab returns dict for observations usually.
        # rsl_rl expects 'obs' to be tensor if simple, or handled by actor_critic.
        
        # Extract policy obs
        policy_obs = obs["policy"]
        return policy_obs, None, rew, dones, infos

    def get_observations(self):
        # Return policy obs
        obs = self.env.reset()[0] # This resets? No!
        # IsaacLab env doesn't usually track current obs in a property suitable for this?
        # Warning: RSL-RL calls step() then uses returned values. 
        # But for initialization?
        # We need to return current obs.
        # HACK: ManagerBasedRLEnv doesn't expose buffer easily?
        # observation_manager.compute() returns result.
        pass
        
    def reset(self):
        obs, _ = self.env.reset()
        return obs["policy"], None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=100)
    args = parser.parse_args()

    # Config
    env_cfg = G1StairsEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    # Set device
    env_cfg.sim.device = "cuda:0"
    
    # Create Env
    env = gym.make("Isaac-Locomotion-G1-v0", cfg=env_cfg) # Task name is just placeholder if cfg provided?
    # Actually gym.make(..., cfg=...) works if registered OR if we use the class directly.
    # But IsaacLab usually registers tasks.
    # We can instantiate ManagerBasedRLEnv directly.
    
    print("[INFO] wrapping environment...")
    # Wrap for RSL-RL
    # IsaacLab usually provides a wrapper `isaaclab_rl.rsl_rl.RslRlVecEnvWrapper`
    # Let's try to import it.
    try:
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        vec_env = RslRlVecEnvWrapper(env)
    except ImportError:
        print("[WARN] Could not import IsaacLab RslRlVecEnvWrapper. Using minimal local one.")
        # Re-implement minimal wrapper logic properly involves handling get_observations.
        # If imports fail, this might fail.
        # For now, let's assume it works or we crash.
        sys.exit(1)

    print("[INFO] Setting up PPO Runner...")
    
    # RSL-RL Config
    # We need to define the agent config (PPO params).
    # Typically loaded from yaml or dict.
    ppo_config = {
        "seed": 42,
        "runner": {
            "policy_class_name": "ActorCritic",
            "algorithm_class_name": "PPO",
            "num_steps_per_env": 24,
            "max_iterations": 1500,
            "save_interval": 50,
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
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "activation": "elu", # or 'elu'
        }
    }
    
    log_dir = os.path.join(script_dir, "logs")
    
    runner = OnPolicyRunner(vec_env, ppo_config, log_dir=log_dir, device=env_cfg.sim.device)
    
    print("[INFO] Starting Training...")
    runner.learn(num_learning_iterations=100, init_at_random_ep_len=True)
    
    print("[INFO] Training Finished.")
    simulation_app.close()

if __name__ == "__main__":
    main()
