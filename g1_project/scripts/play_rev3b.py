
import argparse
import sys
import os
import torch
import traceback
from tensordict import TensorDict
import gymnasium as gym

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

# Launch Isaac Sim (Headless=False for visual)
from isaacsim import SimulationApp
config = {"headless": False} # VISUALIZATION MODE
simulation_app = SimulationApp(config)

# Imports after Sim Start
try:
    from isaaclab.envs import ManagerBasedRLEnv
    from g1_locomotion.g1_rev3b_env_cfg import G1Rev3bEnvCfg # NEW CONFIG
    
    # Import PER Components
    if script_dir not in sys.path:
        sys.path.append(script_dir)
    from per_components import PrioritizedRunner
    
except Exception as e:
    print(f"[ERROR] Import failed: {e}")
    traceback.print_exc()
    sys.exit(1)

# Wrapper (Reused from train_rl_climb.py)
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
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--checkpoint", type=str, default="model_300.pt") # Default to 300
    args = parser.parse_args()

    # Config
    env_cfg = G1Rev3bEnvCfg() # Rev3b
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = "cuda:0" 
    
    # Debug: Print configured USD path
    print(f"[Config] Using Terrain USD: {env_cfg.scene.terrain.usd_path}")
    print(f"[Config] Using Robot USD: {env_cfg.scene.robot.spawn.usd_path}")

    # Disable random initialization for visualization
    # Start facing stairs
    env_cfg.events.reset_base.params["pose_range"] = {"x": (1.0, 1.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}

    # NO STIFFNESS OVERRIDE - Use Config (200.0)
    print("[INFO] Using Config Stiffness (200.0).")

    # Create Env
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Wrap
    vec_env = RslRlVecEnvWrapper(env)
    
    # RSL-RL Config (Minimal for loading)
    ppo_config = {
        "seed": 42,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "num_steps_per_env": 24,
        "max_iterations": 3000,
        "experiment_name": "g1_climb_rev3b",
        "run_name": "run_001",
        "algorithm": { "class_name": "PPO", "value_loss_coef": 1.0, "use_clipped_value_loss": True, "clip_param": 0.2, "entropy_coef": 0.01, "num_learning_epochs": 5, "num_mini_batches": 4, "learning_rate": 1.0e-3, "schedule": "adaptive", "gamma": 0.99, "lam": 0.95, "desired_kl": 0.01, "max_grad_norm": 1.0 },
        "policy": { 
            "class_name": "ActorCritic", 
            "init_noise_std": 1.0, 
            "actor_hidden_dims": [256, 128, 64], 
            "critic_hidden_dims": [256, 128, 64], 
            "activation": "elu",
            "actor_obs_normalization": True, # Enabled!
            "critic_obs_normalization": True,
        }
    }
    
    # Correct Log Dir for Rev3b
    log_dir = os.path.join(script_dir, "logs_rev3b")
    
    # Reset Environment
    vec_env.reset()

    # Use PrioritizedRunner
    runner = PrioritizedRunner(vec_env, ppo_config, log_dir=log_dir, device=env_cfg.sim.device)
    
    # Load model
    resume_path = os.path.join(log_dir, args.checkpoint)
    print(f"[INFO] Loading model from: {resume_path}")
    
    if not os.path.exists(resume_path):
        print(f"[ERROR] Checkpoint not found: {resume_path}")
        sys.exit(1)
        
    # Standard load
    print(f"[INFO] Loading model manually from: {resume_path}")
    loaded_dict = torch.load(resume_path, map_location=env_cfg.sim.device)
    model_state_dict = loaded_dict["model_state_dict"]
    
    # Patch log_std -> std (If needed, likely fixed in newer checkpoints but harmless)
    if "log_std" in model_state_dict and "std" in runner.alg.policy.state_dict():
        print("[INFO] Patching 'log_std' to 'std' in checkpoint.")
        model_state_dict["std"] = model_state_dict.pop("log_std")
        
    runner.alg.policy.load_state_dict(model_state_dict)
    
    # Load Obs Normalization Statistics if available!
    if "actor_obs_normalization" in loaded_dict:
         print("[INFO] Loading Actor Obs Normalization stats.")
         runner.alg.policy.actor_obs_normalization.load_state_dict(loaded_dict["actor_obs_normalization"])
    if "critic_obs_normalization" in loaded_dict:
         print("[INFO] Loading Critic Obs Normalization stats.")
         runner.alg.policy.critic_obs_normalization.load_state_dict(loaded_dict["critic_obs_normalization"])

    policy = runner.alg.policy
    policy.eval()
    
    print("[INFO] Starting Playback...")
    
    obs, count = vec_env.reset()[0], 0
    
    while simulation_app.is_running():
        count += 1
        with torch.no_grad():
            actions = policy.act_inference(obs)
            
            # CLAMP ACTIONS to reasonable range
            actions = torch.clamp(actions, -2.0, 2.0)
            
            obs, rew, dones, extras = vec_env.step(actions)
            
            # DEBUG
            if count % 50 == 0:
                print(f"Frame {count}:")
                print(f"  Actions Max: {actions.abs().max().item():.4f}")
                policy_obs = obs["policy"]
                print(f"  Obs Mean: {policy_obs.mean().item():.4f} Std: {policy_obs.std().item():.4f}")
                
    print("[INFO] Playback Finished.")
    simulation_app.close()

if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
