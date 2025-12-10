import sys
try:
    import pinocchio as pin
    print(f"Pinocchio Version: {str(pin.__version__) if hasattr(pin, '__version__') else 'Unknown'}")
    print(f"File: {pin.__file__}")
    print("Attributes:")
    print(dir(pin))
    
    if hasattr(pin, 'buildModelFromUrdf'):
        print("buildModelFromUrdf FOUND")
    else:
        print("buildModelFromUrdf NOT FOUND")
        
except ImportError as e:
    print(f"ImportError: {e}")
except Exception as e:
    print(f"Error: {e}")
