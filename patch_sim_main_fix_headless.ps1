$file = "C:\Users\basti\source\repos\IsaacLab\source\unitree_sim_isaaclab\sim_main.py"
$content = Get-Content $file -Raw

# Replace the config line
$content = $content -replace "config = \{`"headless`": args_cli.headless\}", "config = {`"headless`": False}"

Set-Content -Path $file -Value $content
Write-Host "Fixed headless arg in sim_main.py"
