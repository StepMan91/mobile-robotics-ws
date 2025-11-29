# PowerShell Script to Launch Verify Asset with Full Environment Setup
# Usage: .\launch_verify.ps1

# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# 2. Add Isaac Sim to PYTHONPATH (Critical for 'import isaacsim')
# We add both the python_packages and the main dir
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH (Critical for Windows)
# This mimics what the launcher does
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH for AppLauncher (just in case we switch back)
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "--------------------------------------------------"
Write-Host "Launching Verify Asset Script with Manual Config"
Write-Host "ISAAC_SIM_PATH: $ENV:ISAAC_SIM_PATH"
Write-Host "PYTHONPATH: $ENV:PYTHONPATH"
Write-Host "--------------------------------------------------"

# 5. Run the script using the Conda Python
# We use the python executable directly to avoid any 'isaaclab.bat' interference
python C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\verify_asset.py
