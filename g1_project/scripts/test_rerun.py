import rerun as rr
import numpy as np
import time

def main():
    rr.init("rerun_test_robotics", spawn=True)
    
    # Log some dummy data to verify 3D viz
    for i in range(100):
        # Log a moving point
        rr.log("world/point", rr.Points3D([np.sin(i*0.1), np.cos(i*0.1), i*0.05], radii=0.1))
        
        # Log a random image
        img = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        rr.log("world/camera/image", rr.Image(img))
        
        time.sleep(0.05)

if __name__ == "__main__":
    main()
