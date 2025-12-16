# Setup Script for Isaac Lab Local Environment (Isaac Sim 5.1)

$isaacSimPath = "C:\isaac-sim"
$repoDir = "C:\Users\basti\source\repos"
$isaacLabDir = "$repoDir\IsaacLab"

Write-Host "Starting Local Environment Setup..." -ForegroundColor Cyan

# 1. Check Isaac Sim Path
if (-not (Test-Path $isaacSimPath)) {
    Write-Host "Error: Isaac Sim not found at $isaacSimPath" -ForegroundColor Red
    exit 1
}
Write-Host "Found Isaac Sim at: $isaacSimPath" -ForegroundColor Green

# 2. Clone Isaac Lab Repository
if (-not (Test-Path $isaacLabDir)) {
    Write-Host "Cloning Isaac Lab repository..."
    git clone https://github.com/isaac-sim/IsaacLab.git $isaacLabDir
}
else {
    Write-Host "Isaac Lab repository already exists." -ForegroundColor Yellow
}

# 3. Instructions for Installation
Write-Host "`nIMPORTANT: Manual Steps Required" -ForegroundColor Magenta
Write-Host "Please execute the following commands in your terminal (Anaconda Prompt recommended):"
Write-Host ""
Write-Host "1. Set the Isaac Sim path:"
Write-Host "   `$env:ISAAC_SIM_PATH = '$isaacSimPath'"
Write-Host ""
Write-Host "2. Navigate to Isaac Lab directory:"
Write-Host "   cd $isaacLabDir"
Write-Host ""
Write-Host "3. Create/Update the Conda environment and install Isaac Lab:"
Write-Host "   .\isaaclab.bat --install"
Write-Host ""
Write-Host "4. Activate the environment:"
Write-Host "   conda activate isaaclab"
Write-Host ""
Write-Host "5. Install your project in editable mode:"
Write-Host "   cd C:\Users\basti\source\repos\mobile-robotics-ws"
Write-Host "   pip install -e g1_project"
Write-Host ""
Write-Host "6. Verify the setup:"
Write-Host "   python g1_project/scripts/verify_asset.py"

Write-Host "`nSetup script finished." -ForegroundColor Cyan
