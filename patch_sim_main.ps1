$file = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab\sim_main.py"
$content = Get-Content $file -Raw

$target = "app_launcher = AppLauncher(args_cli)`r`nsimulation_app = app_launcher.app"
$replacement = "# app_launcher = AppLauncher(args_cli)`r`n# simulation_app = app_launcher.app`r`nfrom isaacsim import SimulationApp`r`nconfig = {`"headless`": args_cli.headless}`r`nsimulation_app = SimulationApp(config)"

# Handle different line endings just in case
$content = $content -replace "app_launcher = AppLauncher\(args_cli\)\r?\n\s*simulation_app = app_launcher.app", $replacement

Set-Content -Path $file -Value $content
Write-Host "Patched sim_main.py"
