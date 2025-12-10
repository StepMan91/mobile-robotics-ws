import sys
import os

print(f"Python executable: {sys.executable}")
# PATCH: Try to set Isaac Vars
if "ISAAC_PATH" not in os.environ and os.path.exists(r"C:\isaac-sim"):
    print("Patching ISAAC_PATH...")
    os.environ["ISAAC_PATH"] = r"C:\isaac-sim"
    # Env vars
    os.environ["EXP_PATH"] = r"C:\isaac-sim\apps"
    os.environ["CARB_APP_PATH"] = r"C:\isaac-sim\kit"
    
    # Critical: Add 'site' to path and import sitecustomize
    site_path = r"C:\isaac-sim\site"
    if site_path not in sys.path:
        sys.path.append(site_path)
        
    try:
        import sitecustomize
        print("Imported sitecustomize successfully.")
    except ImportError as e:
        print(f"Failed to import sitecustomize: {e}")

try:
    import isaacsim
    print(f"isaacsim module: {isaacsim}")
    print(f"isaacsim file: {isaacsim.__file__}")
    print(f"dir(isaacsim): {dir(isaacsim)}")
    
    from isaacsim import SimulationApp
    print(f"SimulationApp imported: {SimulationApp}")
except Exception as e:
    print(f"Error importing isaacsim: {e}")

try:
    import omni.isaac.kit
    print(f"omni.isaac.kit module: {omni.isaac.kit}")
    from omni.isaac.kit import SimulationApp as KitSimulationApp
    print(f"KitSimulationApp: {KitSimulationApp}")
except Exception as e:
    print(f"Error importing omni.isaac.kit: {e}")
