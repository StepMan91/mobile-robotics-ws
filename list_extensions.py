import os

# Path to extscache in the conda environment
# We assume standard path based on previous steps
base_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache"

print(f"Checking extensions in: {base_path}")

if os.path.exists(base_path):
    exts = os.listdir(base_path)
    usd_exts = [e for e in exts if "usd" in e.lower()]
    
    print(f"Found {len(exts)} extensions total.")
    print("USD-related extensions found:")
    for e in usd_exts:
        print(f" - {e}")
else:
    print("extscache directory not found!")
