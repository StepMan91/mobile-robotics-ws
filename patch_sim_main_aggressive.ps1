$file = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab\sim_main.py"
$content = Get-Content $file -Raw

# 1. Comment out import
$content = $content -replace "from isaaclab.app import AppLauncher", "# from isaaclab.app import AppLauncher"

# 2. Comment out argument adding
$content = $content -replace "AppLauncher.add_app_launcher_args\(parser\)", "# AppLauncher.add_app_launcher_args(parser)"

# 3. Replace instantiation (handling my previous patch if it exists or the original)
# Regex to match the block even if I messed it up before
$content = $content -replace "(?s)app_launcher = AppLauncher\(args_cli\).*?simulation_app = app_launcher.app", "# app_launcher = AppLauncher(args_cli)`r`n# simulation_app = app_launcher.app`r`nfrom isaacsim import SimulationApp`r`nconfig = {`"headless`": args_cli.headless}`r`nsimulation_app = SimulationApp(config)"

# 4. Also handle the case where I ALREADY patched it but want to ensure imports are gone
# (The previous regex might fail if I already patched it, so let's be safe and just ensure the import is commented)

Set-Content -Path $file -Value $content
Write-Host "Aggressively patched sim_main.py"
