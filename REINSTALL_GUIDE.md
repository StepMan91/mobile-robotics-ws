# Reinstalling Isaac Sim (Bypassing Launcher)

Since the Omniverse Launcher is broken, we will install Isaac Sim directly using **Isaac Lab** or **pip**.

## Option 1: Use Isaac Lab's Installer (Recommended)

Isaac Lab often comes with a script to handle the installation for you.

1.  Open **PowerShell** (as Administrator).
2.  Navigate to your Isaac Lab folder:
    ```powershell
    cd C:\Users\basti\source\repos\IsaacLab
    ```
3.  Run the install command (this might take a while as it downloads Isaac Sim):
    ```powershell
    .\isaaclab.bat --install
    ```
    *If `--install` isn't recognized, try just `.\isaaclab.bat` and see if it prompts you.*

## Option 2: Manual Pip Installation

If Option 1 fails, we can install Isaac Sim manually into a Conda environment.

1.  **Open Anaconda Prompt** (or PowerShell with conda).
2.  Create a new environment (Python 3.10 is required for Isaac Sim 4.x):
    ```powershell
    conda create -n isaac-sim-env python=3.10
    conda activate isaac-sim-env
    ```
3.  Install Isaac Sim via pip:
    ```powershell
    pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk isaacsim-extscache-kit --extra-index-url https://pypi.nvidia.com
    ```
4.  **Verify Installation**:
    ```powershell
    python -c "from isaacsim import SimulationApp; print('Success!')"
    ```

## After Installation

Once installed, please let me know which method worked.
If you used **Option 1**, Isaac Lab should be ready to go.
If you used **Option 2**, we will need to tell Isaac Lab where the environment is.
