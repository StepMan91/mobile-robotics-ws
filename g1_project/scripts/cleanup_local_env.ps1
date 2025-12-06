# Cleanup Script for Isaac Lab Local Environment

Write-Host "Starting Cleanup..." -ForegroundColor Cyan

# 1. Remove Conda Environment
$envName = "isaaclab"
Write-Host "Removing Conda environment: $envName"
Write-Host "This might take a while..."
conda env remove -n $envName -y
if ($?) {
    Write-Host "Conda environment '$envName' removed." -ForegroundColor Green
} else {
    Write-Host "Failed to remove conda environment (or it doesn't exist)." -ForegroundColor Yellow
}

# 2. Instructions for PATH cleanup
Write-Host "`nIMPORTANT: PATH Cleanup" -ForegroundColor Magenta
Write-Host "Please manually check your User and System Environment Variables and remove entries related to:"
Write-Host " - Isaac Sim"
Write-Host " - Omniverse"
Write-Host " - Any paths pointing to: C:\Users\basti\miniconda3\envs\isaaclab\..."

# 3. Instructions for Cache cleanup
$cachePath = "C:\Users\basti\miniconda3\envs\isaaclab\Lib\site-packages\isaacsim\extscache"
Write-Host "`nCache Cleanup" -ForegroundColor Magenta
Write-Host "You can manually delete the Isaac Sim cache at:"
Write-Host "  $cachePath"
Write-Host "And the Omniverse library folder if you installed it separately."

Write-Host "`nCleanup script finished." -ForegroundColor Cyan
