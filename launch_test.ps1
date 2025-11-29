# PowerShell Script to Launch Test Script with Full Environment Setup

# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:ISAAC_PATH = $ISAAC_SIM_PATH
$ENV:CARB_APP_PATH = "$ISAAC_SIM_PATH\kit"
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"
$UNITREE_REPO_PATH = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab"

# 2. Add Isaac Sim AND Unitree Repo to PYTHONPATH
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ISAAC_SIM_PATH\kit\kernel\py;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH (Critical for Windows)
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH for AppLauncher
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "--------------------------------------------------"
Write-Host "Launching Test Script"
Write-Host "ISAAC_SIM_PATH: $ENV:ISAAC_SIM_PATH"
Write-Host "UNITREE_PATH: $UNITREE_REPO_PATH"
Write-Host "--------------------------------------------------"

# 5. Change to the Unitree directory (To test if this causes the issue)
# Set-Location $UNITREE_REPO_PATH

# 6. Run the test script (using absolute path to where I saved it)
$TEST_SCRIPT = "C:\Users\basti\source\repos\mobile-robotics-ws\test_isaac.py"
& "C:\Users\basti\miniconda3\envs\isaaclab\python.exe" $TEST_SCRIPT
