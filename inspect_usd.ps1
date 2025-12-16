# Set Environment Variables manually to avoid header issues
$ISAAC_SIM_PATH = "C:\isaac-sim"
$ENV:ISAAC_SIM_PATH = $ISAAC_SIM_PATH
$ENV:OMNI_KIT_ACCEPT_EULA = "YES"

# Helper for appending to path
function Append-To-Path {
    param ([string]$Path, [string]$Value)
    if ($Path -eq "" -or $Path -eq $null) { return $Value }
    return "$Value;$Path"
}

# Python Path
$ENV:PYTHONPATH = "$ISAAC_SIM_PATH\python_packages;$ISAAC_SIM_PATH\exts\omni.isaac.python;$ENV:PYTHONPATH"
$ENV:PYTHONPATH = "C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\source;C:\Users\basti\source\repos\IsaacLab\source;C:\Users\basti\source\repos\IsaacLab\source\extensions;C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages;$ENV:PYTHONPATH"

# System Path (DLLs)
$ENV:PATH = "$ISAAC_SIM_PATH;$ISAAC_SIM_PATH\bin;$ISAAC_SIM_PATH\kit;$ISAAC_SIM_PATH\exts\omni.usd.libs\bin;$ISAAC_SIM_PATH\exts\omni.usd.libs\libs;$ENV:PATH"

# Exp Path
$ENV:EXP_PATH = "$ISAAC_SIM_PATH\apps\isaacsim.exp.base.kit"

Write-Host "Launching Inspection Script..."
python C:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\inspect_usd.py > C:\Users\basti\source\repos\mobile-robotics-ws\usd_structure.txt 2>&1
