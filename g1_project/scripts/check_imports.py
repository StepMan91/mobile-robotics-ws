from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

try:
    print("Attempting: from isaacsim.core.api.objects import VisualCuboid")
    from isaacsim.core.api.objects import VisualCuboid
    print("SUCCESS: properties exposed in objects")
except ImportError:
    print("FAILED: properties NOT exposed in objects")

try:
    print("Attempting: from isaacsim.core.api.objects.cuboid import VisualCuboid")
    from isaacsim.core.api.objects.cuboid import VisualCuboid
    print("SUCCESS: properties exposed in objects.cuboid")
except ImportError:
    print("FAILED: properties NOT exposed in objects.cuboid")

try:
    print("Attempting: from isaacsim.core.api.world import World")
    from isaacsim.core.api.world import World
    print("SUCCESS: World exposed in world")
except ImportError:
    print("FAILED: World NOT exposed in world")

simulation_app.close()
