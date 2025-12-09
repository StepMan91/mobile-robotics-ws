# Launch RViz2 for G1 Robot Visualization
# This script sources ROS2 and launches RViz2 with the G1 configuration

Write-Host "=== Launching RViz2 for G1 Visualization ===" -ForegroundColor Cyan

# Source ROS2 Humble
$ROS2_SETUP = "C:\dev\ros2_humble\local_setup.ps1"
if (Test-Path $ROS2_SETUP) {
    Write-Host "Sourcing ROS2 from $ROS2_SETUP" -ForegroundColor Green
    . $ROS2_SETUP
}
else {
    Write-Host "ERROR: ROS2 setup not found at $ROS2_SETUP" -ForegroundColor Red
    exit 1
}

# Get script directory for config path
$SCRIPT_DIR = $PSScriptRoot
$RVIZ_CONFIG = "$SCRIPT_DIR\g1_rviz_config.rviz"

# Check if config exists
if (-not (Test-Path $RVIZ_CONFIG)) {
    Write-Host "WARNING: RViz config not found at $RVIZ_CONFIG" -ForegroundColor Yellow
    Write-Host "Launching RViz2 with default config..." -ForegroundColor Yellow
    rviz2
}
else {
    Write-Host "Using config: $RVIZ_CONFIG" -ForegroundColor Green
    rviz2 -d $RVIZ_CONFIG
}
