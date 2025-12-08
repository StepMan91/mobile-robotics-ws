# Launch Isaac Sim Visualization
# Copied env setup from launch_unitree.ps1

$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:ISAAC_PATH = $ISAAC_SIM_PATH
$ENV:CARB_APP_PATH = "$ISAAC_SIM_PATH\kit"
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps"
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# Source ROS2
$ROS2_SETUP = "C:\dev\ros2_humble\local_setup.ps1"
if (Test-Path $ROS2_SETUP) {
    Write-Host "Sourcing ROS2 from $ROS2_SETUP"
    . $ROS2_SETUP
}
else {
    Write-Host "Warning: ROS2 setup not found. Viz might fail if rclpy is needed."
}

# Add Isaac Sim paths to PYTHONPATH
# Also add IsaacLab source to use AppLauncher
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\site;$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ISAAC_SIM_PATH\kit\kernel\py;$ISAAC_SIM_PATH\exts\isaacsim.simulation_app;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;$ENV:PYTHONPATH"

# Add DLL Paths
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

$SCRIPT_DIR = $PSScriptRoot
$VIZ_SCRIPT = "$SCRIPT_DIR\src\isaac_viz.py"

Write-Host "Launching Isaac Sim Viz..."
# Use the python.exe from Isaac Lab env or specific one that has 'isaacsim' installed.
# Defaulting to system python if it has isaacsim, OR specific conda env.
# launch_unitree.ps1 used: "C:\Users\basti\miniconda3\envs\isaaclab\python.exe"

& "C:\Users\basti\miniconda3\envs\isaaclab\python.exe" $VIZ_SCRIPT
