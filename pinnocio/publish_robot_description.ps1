# Publish Robot Description to ROS2
# RViz2 needs the robot_description topic to display the robot model

Write-Host "=== Publishing G1 Robot Description ===" -ForegroundColor Cyan

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

# Path to G1 URDF
$URDF_PATH = "C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\src\g1_description\urdf\g1.urdf"

if (-not (Test-Path $URDF_PATH)) {
    Write-Host "ERROR: URDF not found at $URDF_PATH" -ForegroundColor Red
    exit 1
}

Write-Host "Reading URDF from: $URDF_PATH" -ForegroundColor Green
$URDF_CONTENT = Get-Content -Path $URDF_PATH -Raw

Write-Host "Publishing to /robot_description topic..." -ForegroundColor Green
ros2 topic pub --once /robot_description std_msgs/msg/String "data: '$URDF_CONTENT'"

Write-Host "Robot description published successfully!" -ForegroundColor Green
Write-Host "You can now launch RViz2 to visualize the robot." -ForegroundColor Cyan
