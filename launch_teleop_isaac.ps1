# Isaac Sim G1 Teleop Launcher
# Orchestrates 3 components:
# 1. PANTIN (C++) -> Captures skeleton, sends UDP to localhost:8888
# 2. RETARGETER (Python) -> Listens UDP:8888, solves IK, publishes ROS2 /joint_states
# 3. ISAAC SIM (Python) -> Listens ROS2, visualizes G1

$ErrorActionPreference = "Stop"

# Paths
$WORKSPACE_ROOT = "C:\Users\basti\source\repos\mobile-robotics-ws"
$PANTIN_EXE = "$WORKSPACE_ROOT\pantin\build\Release\RealsenseBodyPose.exe"
$RETARGETER_SCRIPT = "$WORKSPACE_ROOT\pinnocio\src\main.py"
$ISAAC_VIZ_SCRIPT = "$WORKSPACE_ROOT\pinnocio\src\isaac_viz.py"
$PYTHON_ENV = "$WORKSPACE_ROOT\venv" # Assuming venv exists, or system python

# 1. Start Retargeter (ROS2 Node)
Write-Host ">>> Starting Retargeter (IK Solver)..." -ForegroundColor Cyan
# Ensure ROS2 env is sourced if needed, or rely on python libs
Start-Process -FilePath "python" -ArgumentList "$RETARGETER_SCRIPT", "--mode", "udp" -NoNewWindow
Start-Sleep -Seconds 2

# 2. Start Isaac Sim Visualizer
Write-Host ">>> Starting Isaac Sim..." -ForegroundColor Cyan
# Use the isaacpython.bat wrapper usually provided or just specific python
# Assuming calling the script directly works if environment is set, or use isaac python
$ISAAC_PYTHON = "C:\Users\basti\AppData\Local\ov\pkg\isaac_sim-4.2.0\python.bat" # Verify path!
if (-not (Test-Path $ISAAC_PYTHON)) {
    Write-Warning "Isaac Sim python.bat not found at default location. Using system python (might fail)."
    $ISAAC_PYTHON = "python"
}
Start-Process -FilePath $ISAAC_PYTHON -ArgumentList "$ISAAC_VIZ_SCRIPT" 

# 3. Start Pantin (Camera)
Write-Host ">>> Starting Realsense Tracking..." -ForegroundColor Green
Start-Process -FilePath $PANTIN_EXE -ArgumentList "--model", "$WORKSPACE_ROOT\pantin\models\yolov8n-pose.engine", "--ip", "127.0.0.1", "--port", "8888"

Write-Host ">>> All systems go! Press Ctrl+C to stop this script (child processes may remain)." -ForegroundColor Yellow
