
import sys
import os
import inspect

# Add paths as per train_rl.py
script_dir = os.path.dirname(os.path.abspath(__file__))
# Assuming this script is in g1_project/scripts/tools or similar, adjust if placed in root
# But since we run it from repo root, let's just force the paths we saw in train_rl.py

# logic from train_rl.py
source_dir = os.path.abspath(os.path.join(os.getcwd(), "source"))
sys.path.append(source_dir)

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
    sys.path.append(rsl_rl_path)
    
try:
    from rsl_rl.runners import OnPolicyRunner
    print(f"OnPolicyRunner File: {inspect.getfile(OnPolicyRunner)}")
    print("Methods:")
    for name, method in inspect.getmembers(OnPolicyRunner, predicate=inspect.isfunction):
        print(f"  {name}")
    
    # Also check if we can see the algorithms
    from rsl_rl.algorithms import PPO
    print(f"PPO File: {inspect.getfile(PPO)}")
    

except Exception as e:
    print(f"Error: {e}")
    # Print sys.path to debug
    print("Sys.path:")
    for p in sys.path:
        print(p)
        
    print("\n[INFO] Searching for rsl_rl directory...")
    for root, dirs, files in os.walk(os.getcwd()):
        if "rsl_rl" in dirs:
            print(f"Found rsl_rl at: {os.path.join(root, 'rsl_rl')}")

