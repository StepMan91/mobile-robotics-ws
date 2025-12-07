# Launch Pinnocio Retargeter
# Usage: .\launch_pinnocio.ps1 -mode [udp|csv|bag] -path [optional_path]

param(
    [string]$mode = "udp",
    [string]$path = ""
)

$SCRIPT_DIR = $PSScriptRoot
$SRC_DIR = "$SCRIPT_DIR\src"

# Activate Env ?
# Assuming user has conda env active or we use same python as system
# Using 'python' from path

Write-Host "Launching Pinnocio Retargeter in mode: $mode"
if ($path) {
    python "$SRC_DIR\main.py" --mode $mode --path "$path"
}
else {
    python "$SRC_DIR\main.py" --mode $mode
}
