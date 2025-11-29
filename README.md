# Mobile Robotics Workspace: Unitree G1 RL with Isaac Lab & ROS2

## Chapter 1: Project Overview

### 1.1 Introduction
This project is a comprehensive workspace for developing and training Reinforcement Learning (RL) agents for the **Unitree G1 Humanoid Robot**. It leverages **NVIDIA Isaac Lab** (built on Isaac Sim) for high-fidelity simulation and **ROS2** for robot control and communication. The goal is to train the G1 robot to navigate complex terrains, specifically stairs, using modern RL techniques (PPO) and visualize the training/inference process using **Rerun.io**.

### 1.2 Key Components

*   **Simulation Framework**: [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab) (formerly Orbit).
*   **Robot Platform**: Unitree G1 (29 DOF Humanoid).
*   **Middleware**: ROS2 (Robot Operating System 2).
*   **Visualization**: [Rerun.io](https://rerun.io/) for real-time visualization of agent observations, actions, and rewards.
*   **Reinforcement Learning**:
    *   **Environment**: Custom `G1StairEnvCfg` for stair climbing tasks.
    *   **Algorithm**: PPO (Proximal Policy Optimization) via `rsl_rl` (planned).

### 1.3 Workspace Structure (Current State)
The workspace currently contains a mix of Isaac Lab core files and custom project scripts:
*   **Custom Scripts**:
    *   `g1_init.py`: Registers the custom G1 Gym environment.
    *   `g1_stair_env_cfg.py`: Configuration for the G1 stair climbing environment.
    *   `rl_agent.py`: A ROS2 node serving as the RL agent (currently a placeholder for the control loop).
    *   `my_convert_urdf.py`: A utility script to convert the G1 URDF model to USD format, handling Windows-specific DLL issues.
*   **Core Directories**:
    *   `_isaac_sim`: Isaac Sim installation/links.
    *   `assets`: Contains the G1 robot assets (URDF, USD).
    *   `ros2_ws`: The ROS2 workspace for the agent node.

## Chapter 2: Action Plan

### 2.1 Short-Term Goals (Environment Setup)
1.  **Verify Refactoring**: Ensure all scripts run correctly from their new locations in `g1_project/`.
2.  **Asset Verification**: Confirm the G1 USD asset loads correctly in Isaac Sim using `g1_project/scripts/verify_asset.py`.
3.  **ROS2 Build**: Build the `g1_rl_agent` package in `ros2_ws` to ensure dependencies are met.
# Mobile Robotics Workspace: Unitree G1 RL with Isaac Lab & ROS2

## Chapter 1: Project Overview

### 1.1 Introduction
This project is a comprehensive workspace for developing and training Reinforcement Learning (RL) agents for the **Unitree G1 Humanoid Robot**. It leverages **NVIDIA Isaac Lab** (built on Isaac Sim) for high-fidelity simulation and **ROS2** for robot control and communication. The goal is to train the G1 robot to navigate complex terrains, specifically stairs, using modern RL techniques (PPO) and visualize the training/inference process using **Rerun.io**.

### 1.2 Key Components

*   **Simulation Framework**: [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab) (formerly Orbit).
*   **Robot Platform**: Unitree G1 (29 DOF Humanoid).
*   **Middleware**: ROS2 (Robot Operating System 2).
*   **Visualization**: [Rerun.io](https://rerun.io/) for real-time visualization of agent observations, actions, and rewards.
*   **Reinforcement Learning**:
    *   **Environment**: Custom `G1StairEnvCfg` for stair climbing tasks.
    *   **Algorithm**: PPO (Proximal Policy Optimization) via `rsl_rl` (planned).

### 1.3 Workspace Structure (Current State)
The workspace currently contains a mix of Isaac Lab core files and custom project scripts:
*   **Custom Scripts**:
    *   `g1_init.py`: Registers the custom G1 Gym environment.
    *   `g1_stair_env_cfg.py`: Configuration for the G1 stair climbing environment.
    *   `rl_agent.py`: A ROS2 node serving as the RL agent (currently a placeholder for the control loop).
    *   `my_convert_urdf.py`: A utility script to convert the G1 URDF model to USD format, handling Windows-specific DLL issues.
*   **Core Directories**:
    *   `_isaac_sim`: Isaac Sim installation/links.
    *   `assets`: Contains the G1 robot assets (URDF, USD).
    *   `ros2_ws`: The ROS2 workspace for the agent node.

## Chapter 2: Action Plan

### 2.1 Short-Term Goals (Environment Setup)
1.  **Verify Refactoring**: Ensure all scripts run correctly from their new locations in `g1_project/`.
2.  **Asset Verification**: Confirm the G1 USD asset loads correctly in Isaac Sim using `g1_project/scripts/verify_asset.py`.
3.  **ROS2 Build**: Build the `g1_rl_agent` package in `ros2_ws` to ensure dependencies are met.

### 2.2 Mid-Term Goals (RL Implementation)
1.  **Refine Environment**: Update `g1_stair_env_cfg.py` to tune the stair terrain generation and reward functions specifically for the G1 robot.
2.  **Implement PPO**: Integrate `rsl_rl` or `skrl` to train the PPO agent.
3.  **Bridge ROS2**: Implement the communication bridge to send observations from Isaac Lab to the ROS2 agent and receive actions back (if running inference via ROS2).

### 2.3 Long-Term Goals (Sim-to-Real & Advanced Features)
1.  **Sim-to-Real Transfer**: Validate the trained policy on the physical G1 robot (if available) or a high-fidelity physics check.
2.  **Visual Navigation**: Integrate camera sensors and use a Vision-Language-Action (VLA) model for semantic navigation.
3.  **Rerun Visualization**: Enhance the Rerun integration to visualize internal agent states, value function heatmaps, and reward breakdowns in real-time.

## Chapter 3: Detailed Tutorial

### 3.1 Prerequisites
Before starting, ensure you have the following installed:
*   **OS**: Windows 10/11 (with WSL2 recommended for ROS2) or Linux (Ubuntu 22.04).
*   **GPU**: NVIDIA RTX GPU with latest drivers.
*   **Software**:
    *   [Isaac Sim 4.2+](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
    *   [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
    *   Python 3.10+

### 3.2 Installation & Setup

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/your-username/mobile-robotics-ws.git
    cd mobile-robotics-ws
    ```

2.  **Set up Isaac Lab**:
    Follow the official [Isaac Lab Installation Guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html). Ensure you can run the basic Isaac Lab examples.
    source install/setup.bash
    ```

### 3.3 Running the Simulation

#### Step 1: Verify Assets
Check if the G1 robot loads correctly in Isaac Sim:
```bash
python g1_project/scripts/verify_asset.py
```
This should open Isaac Sim and display the G1 robot standing on a plane.

#### Step 2: Train/Run the Agent
(Note: This is a placeholder command until the RL training script is fully integrated)
```bash
# Example command to launch the environment
python -m isaaclab.scripts.train --task Isaac-Velocity-G1-Stairs-v0 --headless
```

#### Step 3: Run the ROS2 Agent
In a separate terminal (ensure ROS2 is sourced):
```bash
ros2 run g1_rl_agent rl_agent
```
This node will start publishing dummy actions and logging data to Rerun.io.

### 3.4 Workflow Explanation
1.  **Configuration**: Modify `g1_project/config/g1_stair_env_cfg.py` to adjust the robot's physical properties, terrain difficulty, or reward functions.
2.  **Training**: Run the training script (using `rsl_rl` or `skrl`). The policy will be saved as a `.pt` (PyTorch) or `.onnx` file.
3.  **Inference/Deployment**: Load the trained policy and run it in the simulation. The `rl_agent` ROS2 node can be used to bridge this policy to a real robot or for external monitoring.
4.  **Visualization**: Use **Rerun** to inspect the agent's performance. The `rl_agent` node logs data to Rerun, allowing you to see charts of rewards, joint velocities, and other metrics in real-time.

### 3.5 Troubleshooting
*   **DLL Load Failed**: If you encounter DLL errors on Windows, ensure you are running the scripts from the correct Python environment (e.g., the one bundled with Isaac Sim or a conda env with Isaac Sim paths configured). Use `g1_project/scripts/convert_urdf.py` as a reference for setting up DLL paths.
*   **ROS2 Discovery**: If nodes cannot see each other, check your `ROS_DOMAIN_ID` environment variable. It must be the same across all terminals.