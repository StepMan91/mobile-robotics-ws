import sys
print("Isaac Lab Python Environment Verified!")
print(f"Python Version: {sys.version}")
try:
    import omni.isaac.core
    print("Isaac Sim Core found.")
except ImportError:
    print("Isaac Sim Core NOT found.")
