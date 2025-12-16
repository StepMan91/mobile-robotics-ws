# G1 RViz2 Visualization Guide

## Overview
This document provides instructions for visualizing the G1 robot in RViz2 as an alternative to Isaac Sim, which has compatibility issues with Windows ROS2.

## Prerequisites
- ROS2 Humble installed at `C:\dev\ros2_humble`
- G1 URDF model at `C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\src\g1_description\urdf\g1.urdf`
- RViz2 configuration at `g1_rviz_config.rviz` (in this directory)

## Quick Start - Manual Workflow

Due to Python/rclpy compatibility issues on Windows, use this manual multi-terminal workflow:

### Terminal 1: Robot State Publisher
```powershell
# Source ROS2
C:\dev\ros2_humble\local_setup.ps1

# Publish robot description and TF tree
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(Get-Content C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\src\g1_description\urdf\g1.urdf -Raw)"
```

### Terminal 2: RViz2
```powershell
# Source ROS2
C:\dev\ros2_humble\local_setup.ps1

# Launch RViz2 with G1 config
cd C:\Users\basti\source\repos\mobile-robotics-ws\pinnocio
rviz2 -d g1_rviz_config.rviz
```

### Terminal 3: Motion Retargeting Pipeline
```powershell
cd C:\Users\basti\source\repos\mobile-robotics-ws\pinnocio
.\launch_pinnocio.ps1
```

## What You'll See

- **RViz2 Window**: 3D view of the G1 robot
- **Robot Model**: The G1 humanoid robot mesh
- **TF Frames**: Coordinate frames for each joint
- **Joint Motion**: Real-time updates as you run the retargeting pipeline

## Troubleshooting

### Robot doesn't appear
- Verify `robot_state_publisher` is running (Terminal 1)
- Check `/robot_description` topic: `ros2 topic echo /robot_description`

### Joints don't move
- Verify `joint_states` are being published: `ros2 topic echo /joint_states`
- Check that `launch_pinnocio.ps1` is running without errors

### TF frames missing
- Ensure the base frame is set to `pelvis` in RViz2 (should be automatic with provided config)
- Check TF tree: `ros2 run tf2_tools view_frames`

## ROS2 Topics

The system uses these topics:
- `/robot_description` - URDF model (published by robot_state_publisher)
- `/joint_states` - Joint positions (published by main.py retargeter)
- `/tf` - Transform tree (published by robot_state_publisher + main.py)

## Why Not Isaac Sim?

Isaac Sim 5.1's ROS2 bridge on Windows has persistent DLL loading issues with `rmw_fastrtps_cpp.dll`, making it incompatible with external ROS2 installations in this environment. RViz2 is more robust for this use case.
