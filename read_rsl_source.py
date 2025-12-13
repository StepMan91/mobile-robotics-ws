
import os

ppo_path = r"C:\Users\basti\miniconda3\Lib\site-packages\rsl_rl\algorithms\ppo.py"
rsl_dir = r"C:\Users\basti\miniconda3\Lib\site-packages\rsl_rl"

def read_file(path):
    print(f"\n--- Reading {path} ---")
    try:
        with open(path, 'r') as f:
            print(f.read())
    except Exception as e:
        print(f"Error reading {path}: {e}")

read_file(ppo_path)




runner_path = r"C:\Users\basti\miniconda3\Lib\site-packages\rsl_rl\runners\on_policy_runner.py"

with open("runner_dump.txt", "w", encoding="utf-8") as dump:
    dump.write(f"Runner Path: {runner_path}\n")
    try:
        with open(runner_path, 'r', encoding='utf-8') as f:
            dump.write(f.read())
    except Exception as e:
        dump.write(f"Error reading runner: {e}\n")



