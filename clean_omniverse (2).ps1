# PowerShell Script to Clean NVIDIA Omniverse Launcher Cache
# Run this script as Administrator if possible, or just normally.

Write-Host "--- Cleaning NVIDIA Omniverse Launcher Cache ---" -ForegroundColor Cyan

# 1. Close Omniverse Launcher
Write-Host "Closing Omniverse Launcher..."
Stop-Process -Name "Omniverse Launcher" -ErrorAction SilentlyContinue
Stop-Process -Name "omniverse-launcher" -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

# 2. Define Cache Paths
$paths = @(
    "$env:APPDATA\omniverse-launcher",
    "$env:LOCALAPPDATA\ov\cache",
    "$env:USERPROFILE\.nvidia-omniverse\config"
)

# 3. Delete Cache
foreach ($path in $paths) {
    if (Test-Path $path) {
        Write-Host "Removing: $path" -ForegroundColor Yellow
        try {
            Remove-Item -Path $path -Recurse -Force -ErrorAction Stop
            Write-Host "  [OK] Removed." -ForegroundColor Green
        } catch {
            Write-Host "  [ERROR] Could not remove $path. Is the Launcher still open?" -ForegroundColor Red
        }
    } else {
        Write-Host "  [INFO] Path not found (already clean): $path" -ForegroundColor Gray
    }
}

Write-Host "--- Cleanup Complete ---" -ForegroundColor Cyan
Write-Host "Please restart the NVIDIA Omniverse Launcher now."
Write-Host "You will need to log in again."
