$ROS2_SETUP = "C:\dev\ros2_humble\local_setup.ps1"
if (Test-Path $ROS2_SETUP) {
    . $ROS2_SETUP
}
$WS_DIR = "c:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws"
Set-Location $WS_DIR
colcon build --merge-install
