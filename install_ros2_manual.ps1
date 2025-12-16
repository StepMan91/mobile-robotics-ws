# Install ROS2 Humble on Windows
$ErrorActionPreference = "Stop"

$WORK_DIR = "C:\dev"
$ROS2_DIR = "$WORK_DIR\ros2_humble"
$ZIP_URL = "https://github.com/ros2/ros2/releases/download/release-humble-20250721/ros2-humble-20250721-windows-release-amd64.zip"
$ZIP_FILE = "$WORK_DIR\ros2.zip"

# 1. Create Directory
if (-not (Test-Path $WORK_DIR)) {
    New-Item -ItemType Directory -Force -Path $WORK_DIR
}

# 2. Download
Write-Host "Downloading ROS2 form $ZIP_URL..."
Invoke-WebRequest -Uri $ZIP_URL -OutFile $ZIP_FILE -UseBasicParsing

# 3. Extract
Write-Host "Extracting to $ROS2_DIR..."
# Note: The zip usually contains a 'ros2-windows' folder inside.
Expand-Archive -Path $ZIP_FILE -DestinationPath $WORK_DIR -Force

# Rename/Move if needed
if (Test-Path "$WORK_DIR\ros2-windows") {
    if (Test-Path $ROS2_DIR) { Remove-Item -Recurse -Force $ROS2_DIR }
    Rename-Item -Path "$WORK_DIR\ros2-windows" -NewName "ros2_humble"
}

# 4. Cleanup
Remove-Item -Force $ZIP_FILE

Write-Host "ROS2 Installed at $ROS2_DIR"
Write-Host "You generally need to install Visual C++ Redistributable and Python deps."
