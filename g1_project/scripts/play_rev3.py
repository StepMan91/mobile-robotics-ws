# play_rev3.py
# Visualization script specific for Rev3 Training at 2200 Epochs

import argparse
import sys
import os
import traceback

# Add source path
# Match play_g1.py logic
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add IsaacLab path
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
isaaclab_pkg_path = os.path.join(isaac_lab_path, "isaaclab")
if isaaclab_pkg_path not in sys.path:
    sys.path.append(isaaclab_pkg_path)

# Try import SimulationApp
# If this fails with Extension not found, it is an environment issue.
try:
    from isaacsim import SimulationApp
except ImportError:
    print("[ERROR] Could not import isaacsim. Ensure you are running in the correct environment.")
    sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="Play Rev3")
    parser.add_argument("--num_envs", type=int, default=1)
    args = parser.parse_args()

    # Launch App
    config = {"headless": False}
    try:
        simulation_app = SimulationApp(config)
    except Exception as e:
        print(f"[ERROR] Failed to instantiate SimulationApp: {e}")
        # Check if we are incorrectly configured
        traceback.print_exc()
        sys.exit(1)

    # Imports after Sim Start
    try:
        import gymnasium as gym
        from isaaclab.envs import ManagerBasedRLEnv
        from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg
        from tensordict import TensorDict
        import torch
        
        # Import PER Components
        # Need to ensure script_dir in path for per_components
        if script_dir not in sys.path:
            sys.path.append(script_dir)
        from per_components import PrioritizedRunner
        
    except Exception as e:
        print(f"[ERROR] Import failed after sim start: {e}")
        traceback.print_exc()
        simulation_app.close()
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
            policy_obs = obs_dict["policy"]
            
            # Hack: Pad with zeros for missing height_scan
            # Expected 283, got 99
            if policy_obs.shape[1] < 283:
                padding = torch.zeros(policy_obs.shape[0], 283 - policy_obs.shape[1], device=policy_obs.device)
                policy_obs = torch.cat((policy_obs, padding), dim=1)
                
            return TensorDict({"policy": policy_obs}, batch_size=[self.num_envs]), rew, dones, extras

        def get_observations(self):
            policy_obs = self.env.unwrapped.observation_manager.compute()["policy"]
            if policy_obs.shape[1] < 283:
                padding = torch.zeros(policy_obs.shape[0], 283 - policy_obs.shape[1], device=policy_obs.device)
                policy_obs = torch.cat((policy_obs, padding), dim=1)
            return TensorDict({"policy": policy_obs}, batch_size=[self.num_envs])
            
        def reset(self):
            obs_dict, _ = self.env.reset()
            policy_obs = obs_dict["policy"]
            if policy_obs.shape[1] < 283:
                padding = torch.zeros(policy_obs.shape[0], 283 - policy_obs.shape[1], device=policy_obs.device)
                policy_obs = torch.cat((policy_obs, padding), dim=1)
            return TensorDict({"policy": policy_obs}, batch_size=[self.num_envs]), None

        def __getattr__(self, name):
            return getattr(self.env.unwrapped, name)

    # Specific Rev3 Paths
    log_root = os.path.join(script_dir, "logs_per_climb_rev3_7k")
    checkpoint_path = os.path.join(log_root, "model_2200.pt")
    
    if not os.path.exists(checkpoint_path):
        print(f"[ERROR] Checkpoint not found at: {checkpoint_path}")
        simulation_app.close()
        sys.exit(1)

    print(f"[INFO] Loading Checkpoint: {checkpoint_path}")

    # Config
    env_cfg = G1StairsEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0" # Force GPU
    
    # Create Env
    env = gym.make("Isaac-Locomotion-G1-v0", cfg=env_cfg)
    
    # Wrap
    vec_env = RslRlVecEnvWrapper(env)

    # RSL-RL Config
    ppo_config = {
        "seed": 42,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "num_steps_per_env": 24,
        "max_iterations": 7000,
        "save_interval": 50,
        "experiment_name": "g1_stairs_per",
        "run_name": "Rev3", 
        "algorithm": { "class_name": "PPO", "value_loss_coef": 1.0, "use_clipped_value_loss": True, "clip_param": 0.2, "entropy_coef": 0.01, "num_learning_epochs": 5, "num_mini_batches": 4, "learning_rate": 1e-3, "schedule": "adaptive", "gamma": 0.99, "lam": 0.95, "desired_kl": 0.01, "max_grad_norm": 1.0 },
        "policy": { "class_name": "ActorCritic", "init_noise_std": 1.0, "actor_hidden_dims": [128, 64, 32], "critic_hidden_dims": [128, 64, 32], "activation": "elu" }
    }
    
    runner = PrioritizedRunner(vec_env, ppo_config, log_dir=log_root, device=env_cfg.sim.device)
    
    # Load model manually
    # runner.load(checkpoint_path)
    
    print(f"[INFO] Manually loading and patching checkpoint...")
    loaded_dict = torch.load(checkpoint_path, map_location=env_cfg.sim.device)
    
    # RSL-RL saves model in 'model_state_dict'
    if 'model_state_dict' in loaded_dict:
        model_dict = loaded_dict['model_state_dict']
    else:
        model_dict = loaded_dict # Fallback if raw dict
        
    # Patch mismatch keys: 'log_std' -> 'std'
    if "log_std" in model_dict and "std" not in model_dict:
        print("[WARN] Patching checkpoint: Renaming 'log_std' to 'std'")
        model_dict["std"] = model_dict.pop("log_std")
        
    runner.alg.policy.load_state_dict(model_dict)
    
    policy = runner.alg.policy
    policy.eval()
    
    print("[INFO] Starting Playback of Rev3 @ 2200 Epochs...")
    
    obs, _ = vec_env.reset()
    
    while simulation_app.is_running():
        with torch.no_grad():
            actions = policy.act_inference(obs)
            obs, rew, dones, extras = vec_env.step(actions)
            
    print("[INFO] Playback Finished.")
    simulation_app.close()

if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        with open("c:\\Users\\basti\\source\\repos\\mobile-robotics-ws\\error_log.txt", "w") as f:
            traceback.print_exc(file=f)
        sys.exit(1)
