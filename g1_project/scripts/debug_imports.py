
import sys
import os
import traceback

print("[DEBUG] Python Executable:", sys.executable)
print("[DEBUG] Current CWD:", os.getcwd())

# Setup Paths logic (Mirrored from train_rl.py)
script_dir = os.path.dirname(os.path.abspath(__file__))
source_dir = os.path.abspath(os.path.join(script_dir, "../source"))
sys.path.append(source_dir)
print(f"[DEBUG] Appended source: {source_dir}")

isaac_lab_path = r"C:\Users\basti\source\repos\IsaacLab\source"
core_path = os.path.join(isaac_lab_path, "isaaclab")
if core_path not in sys.path:
    sys.path.append(core_path)
    print(f"[DEBUG] Appended core: {core_path}")

ext_path = os.path.join(isaac_lab_path, "extensions")
if ext_path not in sys.path:
    sys.path.append(ext_path)
    print(f"[DEBUG] Appended ext: {ext_path}")

rsl_rl_path = os.path.join(source_dir, "rsl_rl_repo")
if rsl_rl_path not in sys.path:
    sys.path.append(rsl_rl_path)
    print(f"[DEBUG] Appended rsl: {rsl_rl_path}")

print("[DEBUG] Initializing SimulationApp...")
try:
    from isaacsim import SimulationApp
    config = {"headless": True}
    simulation_app = SimulationApp(config)
    print("[DEBUG] SimulationApp Started.")
except Exception:
    print("[FATAL] SimApp Failed")
    traceback.print_exc()
    sys.exit(1)

print("[DEBUG] Importing Libraries...")
try:
    import gymnasium
    print("[PASS] Gymnasium")
    
    import isaaclab
    print(f"[PASS] IsaacLab (Package: {isaaclab.__file__})")
    
    from isaaclab.envs import ManagerBasedRLEnv
    print("[PASS] ManagerBasedRLEnv")
    
    import rsl_rl
    print(f"[PASS] rsl_rl (Package: {rsl_rl.__file__})")
    
    from rsl_rl.runners import OnPolicyRunner
    print("[PASS] OnPolicyRunner")
    
    import g1_locomotion
    print(f"[PASS] g1_locomotion (Package: {g1_locomotion.__file__})")
    
    from g1_locomotion.g1_stairs_env_cfg import G1StairsEnvCfg
    print("[PASS] G1StairsEnvCfg")
    
    print("[SUCCESS] All Imports Passed.")
    
except Exception:
    print("[FATAL] Import Failed during sequence.")
    traceback.print_exc()
    
simulation_app.close()
