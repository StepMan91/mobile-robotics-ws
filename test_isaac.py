from isaacsim import SimulationApp
print("Imported SimulationApp")
config = {"headless": False}
print("Configuring SimulationApp...")
simulation_app = SimulationApp(config)
print("SimulationApp started!")
simulation_app.close()
