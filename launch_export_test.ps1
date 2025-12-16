$env:OMNI_KIT_ACCEPT_EULA = "YES"

# Export Test Env (Cubes+Sphere)
& "C:\isaac-sim\python.bat" "c:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\export_test_env.py"

if ($LASTEXITCODE -ne 0) {
    Write-Host "Export Failed!" -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host "Export Success!" -ForegroundColor Green
