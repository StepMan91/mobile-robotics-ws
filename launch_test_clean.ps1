# PowerShell Script to Launch Test Script with Verified Config

# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# 2. Add Isaac Sim to PYTHONPATH
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH (Critical for Windows)
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "--------------------------------------------------"
Write-Host "Launching Test Script (Clean)"
Write-Host "ISAAC_SIM_PATH: $ENV:ISAAC_SIM_PATH"
Write-Host "--------------------------------------------------"

# 5. Run the test script
python C:\Users\basti\source\repos\mobile-robotics-ws\test_isaac.py
