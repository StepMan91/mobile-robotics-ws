# Launch RViz2 State Publisher for G1 Robot
# This publishes the robot_description and relays joint/TF data for RViz2

Write-Host "=== Launching G1 State Publisher for RViz2 ===" -ForegroundColor Cyan

# Source ROS2 Humble
$ROS2_SETUP = "C:\dev\ros2_humble\local_setup.ps1"
if (Test-Path $ROS2_SETUP) {
    Write-Host "Sourcing ROS2..." -ForegroundColor Green
    . $ROS2_SETUP
}
else {
    Write-Host "ERROR: ROS2 setup not found" -ForegroundColor Red
    exit 1
}

$SCRIPT_DIR = $PSScriptRoot
$PUBLISHER_SCRIPT = "$SCRIPT_DIR\src\rviz2_publisher.py"

if (-not (Test-Path $PUBLISHER_SCRIPT)) {
    Write-Host "ERROR: Publisher script not found at $PUBLISHER_SCRIPT" -ForegroundColor Red
    exit 1
}

Write-Host "Starting robot state publisher..." -ForegroundColor Green
Write-Host "This will publish /robot_description and relay joint_states/tf" -ForegroundColor Cyan
Write-Host "Keep this running while using RViz2" -ForegroundColor Yellow
Write-Host ""

# Use the g1_retargeting conda environment (Python 3.10, compatible with ROS2 Humble)
$CONDA_ENV = "g1_retargeting"
Write-Host "Activating conda environment: $CONDA_ENV" -ForegroundColor Green

# Run via conda
& conda run -n $CONDA_ENV python $PUBLISHER_SCRIPT
