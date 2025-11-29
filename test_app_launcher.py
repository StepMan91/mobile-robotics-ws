from isaaclab.app import AppLauncher
import argparse

print("Initializing AppLauncher...")
args = argparse.Namespace(headless=True, livestream=0)
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app
print("AppLauncher initialized successfully.")
simulation_app.close()
print("App closed.")
