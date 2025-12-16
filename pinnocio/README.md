# Pinnocio - G1 Retargeting Project

This project performs motion retargeting for the Unitree G1 robot using the **Pinocchio** library. It maps human motion from RealSense data (live UDP, CSV, or Rosbag) to robot joint configurations using Inverse Kinematics (IK).

## Project Structure
- **`src/retargeter.py`**: Core IK solver. Maps Human Keypoints (Wrists, Ankles, Pelvis) to Robot Frames.
- **`src/data_loader.py`**: Reads pose data from multiple sources.
- **`src/robot_publisher.py`**: Publishes `JointState` to ROS2.
- **`src/isaac_viz.py`**: Visualizes the robot in Isaac Sim.

## Setup

### Environment
The project uses a Conda environment `g1_retargeting`:
```bash
conda activate g1_retargeting
```

### Dependencies
- `pinocchio` (Rigid Body Dynamics)
- `rosbags` (File reading)
- `numpy`
- `rclpy` (ROS2 Python client - **Must be sourced from global ROS2 install**)

## Usage

### 1. Run the Retargeter
**Important**: You must source your ROS2 workspace before running!

**PowerShell:**
```powershell
# 1. Source ROS2 (Adjust path if needed)
. "C:\Users\basti\source\repos\mobile-robotics-ws\ros2_ws\install\setup.ps1"

# 2. Activate Conda
conda activate g1_retargeting

# 3. Launch
.\launch_pinnocio.ps1 -mode csv -path "..\RealSens_body_Tracking_Human Keypoints\recordings\recording_20251206_205703.csv"
```

**Modes:**
- `-mode udp`: Listens on port 8888 (Live).
- `-mode csv`: Replays a CSV recording.
- `-mode bag`: Replays a Rosbag (requires `rosbags` lib).

### 2. Visualize in Isaac Sim
Open a **separate terminal**:
```powershell
.\launch_viz_sim.ps1
```
This will launch Isaac Sim and animate the G1 robot based on the `joint_states` published by the retargeter.
