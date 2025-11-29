# Reinstalling Isaac Sim & Isaac Lab (Recommended Method)

The manual pip installation encountered persistent "DLL load failed" errors.
Since we have fixed the Python version issue (now using Python 3.10), the **official Isaac Lab installer** is the most robust way to proceed.

## Step 1: Prepare Environment

# Reinstalling Isaac Sim & Isaac Lab (Recommended Method)

The manual pip installation encountered persistent "DLL load failed" errors.
Since we have fixed the Python version issue (now using Python 3.10), the **official Isaac Lab installer** is the most robust way to proceed.

## Step 1: Prepare Environment

1.  Open **Anaconda Prompt**.
*   **Note:** This might take a while as it downloads Isaac Sim again (or verifies existing files).
*   **Prompt:** If asked to install dependencies or accept EULA, type `y` or `yes`.

```powershell
# Run the training test (headless)
# We use Unitree Go2 as a test because G1 is not yet added
python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Unitree-Go2-v0 --headless
```

## Step 3: Verification

Once the installer finishes, run the verification again:

 
 
```

If this starts training (you see FPS numbers), you are done!
