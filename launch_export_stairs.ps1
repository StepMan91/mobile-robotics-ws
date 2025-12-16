# launch_export_stairs.ps1

# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# 2. Add Isaac Sim to PYTHONPATH
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH for AppLauncher
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "--------------------------------------------------"
Write-Host "Regenerating Stairs USD (Collision Enabled)..."
Write-Host "--------------------------------------------------"

# Run script (Headless)
python C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\export_stairs_usd.py
