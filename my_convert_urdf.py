import os
import sys
import glob

# Set EULA
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'YES'

# --- WORKAROUND FOR BROKEN PIP INSTALL ---
extscache_path = r"C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache"

# 1. Add omni.kit.usd.layers to sys.path
layer_dirs = glob.glob(os.path.join(extscache_path, "omni.kit.usd.layers-*"))
if layer_dirs:
    sys.path.append(layer_dirs[0])

# 2. Add omni.usd to sys.path AND add its bin to DLL search path
usd_dirs = glob.glob(os.path.join(extscache_path, "omni.usd-1.*"))
if usd_dirs:
    usd_path = usd_dirs[0]
    sys.path.append(usd_path)
    
    # Add bin to DLL directory (CRITICAL for Windows)
    bin_path = os.path.join(usd_path, "bin")
    if os.path.exists(bin_path):
        os.add_dll_directory(bin_path)
        # Also add to PATH for legacy loading
        os.environ['PATH'] = bin_path + os.pathsep + os.environ['PATH']
    
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
        os.environ['PATH'] = usd_libs_bin + os.pathsep + os.environ['PATH']

# 3. Add omni.kit.usd.collect (dependency of layers?)
collect_dirs = glob.glob(os.path.join(extscache_path, "omni.kit.usd.collect-*"))
if collect_dirs:
    sys.path.append(collect_dirs[0])

# -----------------------------------------

try:
    from isaacsim import SimulationApp
    
    # Start the app (headless)
    config = {
        "headless": True,
        "extra_ext_paths": [extscache_path]
    }
    simulation_app = SimulationApp(config)
    print("SimulationApp started successfully.")
    
    # Now import Isaac Lab tools
    from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
    
    # Define paths
    # Use absolute paths to be safe
    cwd = os.getcwd()
    input_urdf = os.path.join(cwd, "assets/unitree_ros/robots/g1_description/g1_29dof.urdf")
    output_usd = os.path.join(cwd, "assets/g1_29dof.usd")
    
    print(f"Converting: {input_urdf}")
    print(f"To: {output_usd}")
    
    # Configure converter
    cfg = UrdfConverterCfg(
        asset_path=input_urdf,
        usd_dir=os.path.dirname(output_usd),
        usd_file_name=os.path.basename(output_usd),
        fix_base=False,
        merge_fixed_joints=False,
        force_usd_paths_as_relative=True,
    )
    
    converter = UrdfConverter(cfg)
    converter.convert()
    
    print("Conversion finished!")
    simulation_app.close()

except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"Error: {e}")
