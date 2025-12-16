# Debug Script
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# Match launch_verify.ps1 PYTHONPATH
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"

# PATH
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# EXP_PATH
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "Checking Python Version..."
& "C:\Users\basti\miniconda3\envs\isaaclab\python.exe" --version

Write-Host "Running verify_asset.py with isaaclab env..."
& "C:\Users\basti\miniconda3\envs\isaaclab\python.exe" C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\verify_asset.py --headless
