# Manual RViz2 Visualization Workflow for G1 Robot
# Since automated Python scripts have rclpy compatibility issues, use this manual workflow

Write-Host "=== G1 RViz2 Visualization - Manual Workflow ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "Due to Windows ROS2/Python compatibility issues, please follow these steps MANUALLY:" -ForegroundColor Yellow
Write-Host ""

Write-Host "STEP 1: Source ROS2 Environment" -ForegroundColor Green
Write-Host "  Open a NEW PowerShell terminal and run:" -ForegroundColor White
Write-Host "  > C:\dev\ros2_humble\local_setup.ps1" -ForegroundColor Cyan
Write-Host"" 

Write-Host "STEP 2: Publish Robot Description" -ForegroundColor Green
Write-Host "  In the SAME terminal, run:" -ForegroundColor White
Write-Host "  > ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=`"`$(Get-Content C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\src\g1_description\urdf\g1.urdf -Raw)`"" -ForegroundColor Cyan
Write-Host ""

Write-Host "STEP 3: Launch RViz2" -ForegroundColor Green
Write-Host "  Open ANOTHER PowerShell terminal and run:" -ForegroundColor White
Write-Host "  > C:\dev\ros2_humble\local_setup.ps1" -ForegroundColor Cyan
Write-Host "  > cd C:\Users\basti\source\repos\mobile-robotics-ws\pinnocio" -ForegroundColor Cyan
Write-Host "  > rviz2 -d g1_rviz_config.rviz" -ForegroundColor Cyan
Write-Host ""

Write-Host "STEP 4: Run Your Retargeting Pipeline" -ForegroundColor Green
Write-Host "  Open another terminal and run:" -ForegroundColor White
Write-Host "  > cd C:\Users\basti\source\repos\mobile-robotics-ws\pinnocio" -ForegroundColor Cyan
Write-Host "  > .\launch_pinnocio.ps1" -ForegroundColor Cyan
Write-Host ""

Write-Host "The G1 robot should now be visible in RViz2 and update based on your retargeting data!" -ForegroundColor Green
Write-Host ""
Write-Host "TROUBLESHOOTING:" -ForegroundColor Yellow
Write-Host "  - If robot doesn't appear: Check that robot_state_publisher is running" -ForegroundColor White
Write-Host "  - If joints don't move: Check that joint_states topic is being published" -ForegroundColor White
Write-Host "  - Use 'ros2 topic list' to see active topics" -ForegroundColor White
Write-Host "  - Use 'ros2 topic echo /robot_description' to verify robot model" -ForegroundColor White
Write-Host ""
