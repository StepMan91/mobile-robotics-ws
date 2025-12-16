# PowerShell Script to Launch Unitree Sim with Full Environment Setup
# Usage: .\launch_unitree.ps1 [args]

# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:ISAAC_PATH = $ISAAC_SIM_PATH
$ENV:CARB_APP_PATH = "$ISAAC_SIM_PATH\kit"
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"
$UNITREE_REPO_PATH = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab"

# 2. Add Isaac Sim AND Unitree Repo to PYTHONPATH
# Added kit\kernel\py and isaacsim.simulation_app based on python.bat analysis
# Added site at the beginning to trigger sitecustomize.py for DLL loading
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\site;$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ISAAC_SIM_PATH\kit\kernel\py;$ISAAC_SIM_PATH\exts\isaacsim.simulation_app;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH (Critical for Windows)
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH for AppLauncher
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "--------------------------------------------------"
Write-Host "Launching Unitree Sim Environment"
Write-Host "ISAAC_SIM_PATH: $ENV:ISAAC_SIM_PATH"
Write-Host "UNITREE_PATH: $UNITREE_REPO_PATH"
Write-Host "--------------------------------------------------"

# 5. Change to the Unitree directory
# Set-Location $UNITREE_REPO_PATH

# 6. Run the main script with any arguments passed to this script
# Use specific Conda Python to avoid picking up Isaac Sim's internal python
& "C:\Users\basti\miniconda3\envs\isaaclab\python.exe" "C:\Users\basti\source\repos\IsaacLab\test_isaac_root.py" $args
