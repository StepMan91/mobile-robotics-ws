import os

base_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim"
search_str = "omni.kit.usd"

print(f"Searching for '{search_str}' in {base_path}...")

for root, dirs, files in os.walk(base_path):
    for file in files:
        if file.endswith(".py"):
            path = os.path.join(root, file)
            try:
                with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    if search_str in content:
                        print(f"Found in: {path}")
                        for line in content.splitlines():
                            if search_str in line:
                                print(f"  Line: {line.strip()}")
            except Exception as e:
                print(f"Error reading {path}: {e}")

# Also verify utils.py specifically
utils_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\exts\isaacsim.simulation_app\isaacsim\simulation_app\utils.py"
print(f"\nVerifying {utils_path} content:")
try:
    with open(utils_path, 'r') as f:
        content = f.read()
        if "import omni.kit.usd.layers as layers" in content:
            print("  PATCH VERIFIED: Found 'import omni.kit.usd.layers as layers'")
        else:
            print("  PATCH FAILED: Did not find expected import.")
        
        if "from omni.kit.usd import layers" in content:
             print("  WARNING: Found old import 'from omni.kit.usd import layers'")
except Exception as e:
    print(f"Error: {e}")
