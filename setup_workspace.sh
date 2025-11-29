#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Create Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Install Python Requirements
# We use the system python for ROS2 compatibility, but install user packages in user site or venv.
# For simplicity with ROS2, we'll install to user site or use a venv that includes system site packages.
# Let's use pip install --user for now to avoid venv complexity with ROS2 unless necessary.
pip3 install --user rerun-sdk torch gymnasium stable-baselines3

# Build Workspace
colcon build --symlink-install

echo "--- Workspace Setup Complete ---"
