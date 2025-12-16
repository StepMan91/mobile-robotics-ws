# Launch Isaac Sim Visualization
# Copied env setup from launch_unitree.ps1

$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:ISAAC_PATH = $ISAAC_SIM_PATH
$ENV:CARB_APP_PATH = "$ISAAC_SIM_PATH\kit"
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps"
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"


# Source ROS2 - REMOVED to avoid conflict with Isaac Sim's bundled ROS2 (Python 3.11)
# Isaac Sim has its own bundled ROS2 bridge which will be used.
# PER LOG INSTRUCTIONS from Isaac Sim 5.1:
$ENV:ROS_DISTRO = "humble"
$ENV:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
# Add bundled ROS2 lib to PATH (Critical for DLL loading)
$ENV:PATH = "$ENV:PATH;C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"

# Add Isaac Sim paths to PYTHONPATH
# Also add bundled rclpy to PYTHONPATH explicitly
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\site;$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ISAAC_SIM_PATH\kit\kernel\py;$ISAAC_SIM_PATH\exts\isaacsim.simulation_app;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\rclpy;$ENV:PYTHONPATH"

# Add DLL Paths
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

$SCRIPT_DIR = $PSScriptRoot
$VIZ_SCRIPT = "$SCRIPT_DIR\src\isaac_viz.py"

Write-Host "Launching Isaac Sim Viz using internal Python..."
# Use Isaac Sim's bundled wrapper which sets up the full environment correctly
& "$ISAAC_SIM_PATH\python.bat" $VIZ_SCRIPT
