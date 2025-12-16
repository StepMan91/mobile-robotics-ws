# 1. Set Critical Paths
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# 2. Add Isaac Sim to PYTHONPATH (Match launch_verify.ps1)
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"

# 3. Add DLL Paths to PATH (Critical for Windows)
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# 4. Set EXP_PATH for AppLauncher
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

# 5. Add our source directory AND Isaac Lab to PYTHONPATH
$ENV:PYTHONPATH = "C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\source;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages;$ENV:PYTHONPATH"

# 6. Run List Joints
Write-Host "--------------------------------------------------"
Write-Host "Listing Joints..."
Write-Host "--------------------------------------------------"

python C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\list_joints_rev4.py --headless > joints.txt 2>&1
