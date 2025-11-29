import os
import sys
import glob

# Set EULA
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'YES'

# --- WORKAROUND FOR BROKEN PIP INSTALL ---
# The pip install of isaacsim 4.5.0 seems to miss adding some extension paths and DLL paths.
# We manually add them here before importing isaacsim.

extscache_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache"

print(f"Applying path workarounds using extscache: {extscache_path}")

# 1. Add omni.kit.usd.layers to sys.path
layer_dirs = glob.glob(os.path.join(extscache_path, "omni.kit.usd.layers-*"))
if layer_dirs:
    sys.path.append(layer_dirs[0])
    print(f"Added to sys.path: {layer_dirs[0]}")

# 2. Add omni.usd to sys.path AND add its bin to DLL search path
usd_dirs = glob.glob(os.path.join(extscache_path, "omni.usd-1.*"))
if usd_dirs:
    usd_path = usd_dirs[0]
    sys.path.append(usd_path)
    print(f"Added to sys.path: {usd_path}")
    
    # Add bin to DLL directory (CRITICAL for Windows)
    bin_path = os.path.join(usd_path, "bin")
    if os.path.exists(bin_path):
        os.add_dll_directory(bin_path)
        print(f"Added to DLL directory: {bin_path}")
    
    # Also check for libs folder just in case
    libs_path = os.path.join(usd_path, "libs")
    if os.path.exists(libs_path):
        os.add_dll_directory(libs_path)

# 2b. Add omni.usd.libs bin to DLL directory (CRITICAL)
usd_libs_dirs = glob.glob(os.path.join(extscache_path, "omni.usd.libs-*"))
if usd_libs_dirs:
    usd_libs_path = usd_libs_dirs[0]
    usd_libs_bin = os.path.join(usd_libs_path, "bin")
    if os.path.exists(usd_libs_bin):
        os.add_dll_directory(usd_libs_bin)
        # Also add to PATH for legacy loading
        os.environ['PATH'] = usd_libs_bin + os.pathsep + os.environ['PATH']
        print(f"Added to DLL directory and PATH: {usd_libs_bin}")

# 3. Add omni.kit.usd.collect (dependency of layers?)
collect_dirs = glob.glob(os.path.join(extscache_path, "omni.kit.usd.collect-*"))
if collect_dirs:
    sys.path.append(collect_dirs[0])

# -----------------------------------------

try:
    from isaacsim import SimulationApp
    print("Isaac Sim (SimulationApp) found.")
    
    # Start the app (headless)
    # We also pass extscache path to Kit just in case
    config = {
        "headless": True,
        "extra_ext_paths": [extscache_path]
    }
    
    simulation_app = SimulationApp(config)
    print("SimulationApp started successfully.")
    
    import omni.isaac.lab
    print("Isaac Lab imported successfully.")
    
    simulation_app.close()
    print("Verification Complete!")
    
except ImportError as e:
    import traceback
    traceback.print_exc()
    print(f"Import Error: {e}")
except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"Runtime Error: {e}")
