# Launch Pinnocio Retargeter
# Usage: .\launch_pinnocio.ps1 -mode [udp|csv|bag] -path [optional_path]

param(
    [string]$mode = "udp",
    [string]$path = ""
)

$SCRIPT_DIR = $PSScriptRoot
$SRC_DIR = "$SCRIPT_DIR\src"

$ROS2_SETUP = "C:\dev\ros2_humble\local_setup.ps1"
if (Test-Path $ROS2_SETUP) {
    Write-Host "Sourcing ROS2 from $ROS2_SETUP"
    . $ROS2_SETUP
}
else {
    Write-Host "Warning: ROS2 setup not found at $ROS2_SETUP. Ensure ROS2 is sourced."
}

# Activate Env ?
# Assuming user has conda env active or we use same python as system
# Using 'python' from path

Write-Host "Launching Pinnocio Retargeter in mode: $mode"
if ($path) {
    python "$SRC_DIR\main.py" --mode $mode --path "$path"
}
else {
    python "$SRC_DIR\main.py" --mode $mode
}
