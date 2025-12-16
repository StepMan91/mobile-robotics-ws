# Verification Launch Script
# Mirrors launch_viz_sim.ps1 environment setup

$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:ISAAC_PATH = $ISAAC_SIM_PATH
$ENV:CARB_APP_PATH = "$ISAAC_SIM_PATH\kit"
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps"
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# PER LOG INSTRUCTIONS from Isaac Sim 5.1:
$ENV:ROS_DISTRO = "humble"
$ENV:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
# Add bundled ROS2 lib to PATH (Critical for DLL loading)
$ENV:PATH = "$ENV:PATH;C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"

# Add Isaac Sim paths to PYTHONPATH
# Also add bundled rclpy to PYTHONPATH explicitly so import works immediately
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\site;$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ISAAC_SIM_PATH\kit\kernel\py;$ISAAC_SIM_PATH\exts\isaacsim.simulation_app;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\rclpy;$ENV:PYTHONPATH"

# Add DLL Paths
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

$SCRIPT_DIR = $PSScriptRoot
# Target the clean test script
$VIZ_SCRIPT = "$SCRIPT_DIR\test_viz_headless.py"

Write-Host "Launching Verification (Clean Env)..."
# Use Isaac Sim's bundled wrapper
& "$ISAAC_SIM_PATH\python.bat" $VIZ_SCRIPT
