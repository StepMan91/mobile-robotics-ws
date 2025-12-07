# Mobile Robotics Workspace: Unitree G1 RL with Isaac Lab & ROS2

## Project Overview

This workspace is a comprehensive development environment for **Reinforcement Learning (RL)** and **Robotic Control** centered around the **Unitree G1 Humanoid Robot**. It integrates **NVIDIA Isaac Lab** for high-fidelity physics simulation with **ROS2** for middleware communication, enabling a seamless sim-to-real workflow.

The primary objective is to develop and train robust RL policies for humanoid locomotion (specifically stair climbing) and visualize agent performance in real-time using **Rerun.io**.

### Key Technologies
*   **Simulation**: [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab) (based on Isaac Sim 4.2+).
*   **Robot Platform**: Unitree G1 (29 DOF Humanoid).
*   **Middleware**: ROS2 Humble (Communication Bridge & Control).
*   **Visualization**: [Rerun.io](https://rerun.io/) (Real-time telemetry & 3D viz).
*   **RL Framework**: PPO (via `rsl_rl` or `skrl`).

---

## Directory Structure

| Directory | Description |
| :--- | :--- |
| **`g1_project/`** | **Core Python logic**. Contains the custom Gymnasium environments (`envs/`), configuration files (`config/`), and main execution scripts (`scripts/`). |
| **`ros2_ws/`** | **ROS2 Workspace**. Contains the ROS nodes for bridging simulation data and running the RL agent. |
| &nbsp;&nbsp;`src/g1_description` | URDF, meshes, and visual assets for the Unitree G1. |
| &nbsp;&nbsp;`src/ros2_bridge` | Handles UDP <-> ROS2 communication (e.g., for body pose data). |
| &nbsp;&nbsp;`src/g1_rl_agent` | The RL agent node (inference & control loop). |
| **`source/`** | **Isaac Lab Extensions**. Contains the source code for Isaac Lab tasks and assets. |
| **`_isaac_sim/`** | Symbolic link/folder for the Isaac Sim installation reference. |
| **`assets/`** | Location for storing heavy assets (USD, GLB files). |
| **`scripts/`** | Helper utilities for asset conversion and validation. |

---

## Getting Started

### Prerequisites
*   **OS**: Windows 10/11 (PowerShell) or Linux (Ubuntu 22.04).
*   **Accerator**: NVIDIA RTX GPU with latest drivers.
*   **Software**: Isaac Sim 4.2+, ROS2 Humble, Python 3.10.

### Installation
1.  **Clone the repository**:
    ```bash
    git clone <repo-url>
    cd mobile-robotics-ws
    ```
2.  **Environment Setup**:
    Ensure `conda` is installed and the `isaaclab` environment is configured.
    (See `g1_project/README.md` for detailed package installation if available).

---

## Usage (Windows PowerShell)

We provide PowerShell wrapper scripts to handle the complex environment variable setup required for Isaac Sim on Windows.

### 1. Run Simulation / Inference
To launch the G1 robot in the Isaac Lab environment and run the default policy:
```powershell
.\launch_play.ps1
```
*   *Under the hood*: Sets env vars -> runs `g1_project/scripts/play_g1.py`.

### 2. Train the Agent (RL)
To start a PPO training session:
```powershell
.\launch_train.ps1
```
*   *Note*: Ensure your training parameters are set in `g1_project/config/g1_stair_env_cfg.py`.

### 3. Debugging & Testing
*   **Verify Asset Loading**: `.\launch_verify.ps1`
*   **Unitree Sim Test**: `.\launch_unitree.ps1`
*   **General Debug**: `.\launch_debug.ps1`

---

## ROS2 Bridge Usage

To use the ROS2 bridge for communicating data (e.g., Skeleton/Body Pose from external cameras):

1.  **Build the Workspace** (in a ROS2-enabled terminal):
    ```bash
    cd ros2_ws
    colcon build
    source install/setup.ps1  # or setup.bash
    ```
2.  **Run the Bridge Node**:
    ```bash
    ros2 run ros2_bridge human_bridge_node
    ```
