import sys
import os

# Mock environment to simulate being inside Kit (simplified)
# We just want to see if python can find the module if we add the path
# But wait, these are extensions, they are not in site-packages directly usually?
# In pip install, they ARE in site-packages/isaacsim/extscache/...
# But they are not added to sys.path automatically until Kit enables them.

# However, let's try to see if we can import it if we manually add the path
# This helps confirm if the folder content is valid python package.

# Found path from previous step:
# Lib\site-packages\isaacsim\extscache\omni.kit.usd.layers-2.2.0+d02c707b.wx64.r.cp310

layer_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache\omni.kit.usd.layers-2.2.0+d02c707b.wx64.r.cp310"

sys.path.append(layer_path)

try:
    import omni.kit.usd.layers
    print("Success: Imported omni.kit.usd.layers")
except ImportError as e:
    print(f"Failed to import omni.kit.usd.layers: {e}")

try:
    from omni.kit.usd import layers
    print("Success: Imported layers from omni.kit.usd")
except ImportError as e:
    print(f"Failed to import from omni.kit.usd: {e}")
