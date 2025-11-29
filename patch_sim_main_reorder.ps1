$file = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab\sim_main.py"
$content = Get-Content $file -Raw

# 1. Remove existing import pinocchio (we will add it back later)
$content = $content -replace "import pinocchio\s*", ""

# 2. Clean up the mess I made (duplicates)
# Remove the whole block of my previous patches
$content = $content -replace "(?s)# # app_launcher = AppLauncher\(args_cli\).*?simulation_app = SimulationApp\(config\)\s*", ""
# Also remove any lingering single copy if the above regex missed
$content = $content -replace "(?s)from isaacsim import SimulationApp.*?simulation_app = SimulationApp\(config\)\s*", ""

# 3. Insert the clean block
# We find the place where AppLauncher WAS (or the commented out version)
# and insert our clean block there.
$clean_block = "
# --- PATCHED BY GEMINI ---
# Ensure SimulationApp is started BEFORE other imports
from isaacsim import SimulationApp
config = {`"headless`": False}
simulation_app = SimulationApp(config)
import pinocchio # Moved here
# -------------------------
"

# We look for the commented out AppLauncher line
if ($content -match "# AppLauncher.add_app_launcher_args\(parser\)") {
    # Insert after args_cli = parser.parse_args()
    $content = $content -replace "(args_cli = parser.parse_args\(\))", "`$1`r`n$clean_block"
}
else {
    # Fallback: Insert after imports if we can't find the args line
    Write-Host "Could not find insertion point, appending to top..."
    $content = $clean_block + $content
}

Set-Content -Path $file -Value $content
Write-Host "Reordered imports in sim_main.py"
