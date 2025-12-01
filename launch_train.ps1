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
# We need Isaac Lab source for omni.isaac.lab (or isaaclab package)
# Adding source/extensions so that 'omni.isaac.lab' can be imported if needed
# ALSO adding isaaclab env site-packages to find gymnasium etc.
$ENV:PYTHONPATH = "C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\source;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages;$ENV:PYTHONPATH"

Write-Host "--------------------------------------------------"
Write-Host "Launching G1 Training Script (Base Env)"
Write-Host "ISAAC_SIM_PATH: $ENV:ISAAC_SIM_PATH"
Write-Host "PYTHONPATH: $ENV:PYTHONPATH"
Write-Host "--------------------------------------------------"

# 6. Run the script using Base Python
python C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\train_g1.py $args
