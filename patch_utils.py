import os

file_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\exts\isaacsim.simulation_app\isaacsim\simulation_app\utils.py"

print(f"Patching {file_path}...")

try:
    with open(file_path, 'r') as f:
        content = f.read()
    
    # The problematic line
    old_import = "from omni.kit.usd import layers"
    new_import = "import omni.kit.usd.layers as layers"
    
    if old_import in content:
        new_content = content.replace(old_import, new_import)
        with open(file_path, 'w') as f:
            f.write(new_content)
        print("Success: File patched!")
    else:
        print("Warning: Target import string not found. File might be already patched or different version.")
        # Debug: print lines containing 'omni.kit.usd'
        print("Lines containing 'omni.kit.usd':")
        for line in content.splitlines():
            if "omni.kit.usd" in line:
                print(line.strip())

except Exception as e:
    print(f"Error: {e}")
