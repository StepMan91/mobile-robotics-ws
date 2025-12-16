import os
import sys

# Add paths manually
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

# RSL_RL
rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaaclab.envs import ManagerBasedRLEnv
from g1_locomotion.g1_rev4_env_cfg import G1Rev4EnvCfg
import torch

def main():
    cfg = G1Rev4EnvCfg()
    cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=cfg)
    
    # Get robot
    robot = env.scene["robot"]
    print("\n[INFO] JOINTS FOUND:")
    print(robot.data.joint_names)
    
    simulation_app.close()

if __name__ == "__main__":
    main()
