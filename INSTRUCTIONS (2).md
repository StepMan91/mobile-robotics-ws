# Step-by-Step Instructions

## 1. Troubleshooting Omniverse Launcher

### Issue: Login Error or "TypeError: o is not a function"
If you see a JavaScript error or cannot login, your Launcher cache is likely corrupted.

**Fix: Clear Cache**
1.  Open **PowerShell**.
2.  Run the cleanup script I created:
    ```powershell
    C:\Users\basti\.gemini\antigravity\playground\cobalt-cosmos\clean_omniverse.ps1
    ```
3.  **Restart** the Omniverse Launcher.
4.  **Log in** again.

### Issue: "Authorization flow not allowed"
If you still see this after cleaning cache:
1.  Open Chrome/Edge.
2.  Go to `chrome://flags` (or `edge://flags`).
3.  Disable `Block insecure private network requests`.
4.  Restart browser.

## 2. Running the Scene in Isaac Sim

1.  **Launch Isaac Sim**: Open the **NVIDIA Omniverse Launcher**, go to the **Library** tab, and launch **Isaac Sim**.
2.  **Open Script Editor**:
    *   In the top menu bar, click on `Window`.
    *   Select `Script Editor`.
3.  **Load the Script**:
    *   Open the file `c:\Users\basti\.gemini\antigravity\playground\cobalt-cosmos\isaac_sim_scripts\generate_env.py` in your code editor (VS Code).
    *   Copy the **entire content** of the file.
    *   Paste it into the **Isaac Sim Script Editor** window.
4.  **Run the Script**:
    *   Click the **Run** button (or press `Ctrl+Enter`) in the Script Editor.
    *   **Result**: You should see a staircase appear in the viewport.
5.  **Find Your Robot Asset**:
    *   In the **Content** window (usually at the bottom), navigate to where your Unitree G1 USD file is located.
    *   Right-click on the `.usd` file.
    *   Select **Copy Path**.
    *   **Paste this path** in the chat to me.

## 3. Running the RL Agent in WSL2

1.  **Open WSL Terminal**:
    *   Open **PowerShell** or **Command Prompt**.
    *   Type `wsl -d Ubuntu-22.04` and press Enter.
2.  **Run the Agent**:
    *   Copy and paste the following block of commands:
        ```bash
        cd ~/ros2_ws
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 run unitree_rl rl_agent
        ```
3.  **Verify Rerun**:
    *   A new window (Rerun Viewer) should open automatically.
    *   If it opens, you should see graphs and a 3D view updating.
