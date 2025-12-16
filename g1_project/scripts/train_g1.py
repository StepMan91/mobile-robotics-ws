import argparse
import sys
import os

# Add the source directory to sys.path so we can import g1_locomotion
# This assumes the script is run from mobile-robotics-ws
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)

# Add Isaac Lab source paths
isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
sys.path.append(os.path.join(isaac_lab_path, "isaaclab"))

# DEBUG: Print sys.path
print("DEBUG: sys.path:")
for p in sys.path:
    print(p)

try:
    from isaacsim import SimulationApp
    print(f"SUCCESS: Imported SimulationApp: {SimulationApp}")
except ImportError as e:
    print(f"FAILED: Import SimulationApp: {e}")

try:
    from isaaclab.app import AppLauncher
    print("SUCCESS: Imported AppLauncher from isaaclab.app")
except ImportError as e:
    print(f"FAILED: Import AppLauncher from isaaclab.app: {e}")
    sys.exit(1)

# Parse arguments
parser = argparse.ArgumentParser(description="Train G1 Locomotion")
parser.add_argument("--video", action="store_true", default=False, help="Record video")
parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode")
parser.add_argument("--task", type=str, default="Isaac-Locomotion-G1-v0", help="Task name")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric")

args_cli = parser.parse_args()

# Launch the app manually
config = {"headless": args_cli.headless}
simulation_app = SimulationApp(config)

# Import extensions after starting the app
import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedRLEnv
import isaaclab.envs.mdp as mdp

# Import the task to register it
import g1_locomotion
from g1_locomotion.g1_env_cfg import G1LocomotionEnvCfg


def main():
    # Instantiate configuration directly
    print("DEBUG: Instantiating G1LocomotionEnvCfg directly...")
    env_cfg = G1LocomotionEnvCfg()
    
    # Apply overrides
    if args_cli.num_envs:
        env_cfg.scene.num_envs = args_cli.num_envs
    
    # Handle device placement (ManagerBasedRLEnv handles this based on sim config usually)
    # But we might need to set it if we want to force CPU/GPU
    if args_cli.cpu:
        env_cfg.sim.device = "cpu"
        env_cfg.sim.use_gpu_pipeline = False
    else:
        env_cfg.sim.device = "cuda:0"
        env_cfg.sim.use_gpu_pipeline = True
        
    if args_cli.disable_fabric:
        env_cfg.sim.use_fabric = False

    print(f"DEBUG: env_cfg type: {type(env_cfg)}")
    
    # Create the environment
    # We pass the config object directly
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    
    print(f"[INFO] Environment created: {env.unwrapped.cfg.scene.num_envs} environments")
    
    # Reset the environment
    obs, _ = env.reset()
    
    # Simple random agent loop
    print("[INFO] Starting simulation loop...")
    while simulation_app.is_running():
        # Sample random actions
        actions = torch.rand((env.unwrapped.num_envs, env.unwrapped.action_space.shape[1]), device=env.unwrapped.device) * 2 - 1
        
        # Step the environment
        obs, rew, terminated, truncated, info = env.step(actions)
        
    # Close the environment
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
