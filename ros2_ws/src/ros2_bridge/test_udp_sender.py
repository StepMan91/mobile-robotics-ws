import socket
import json
import time
import math

UDP_IP = "172.31.69.131"
UDP_PORT = 8888

print(f"UDP Target IP: {UDP_IP}")
print(f"UDP Target Port: {UDP_PORT}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    frame = 0
    while True:
        t = frame * 0.1
        
        # Simulate Wrist Motion
        payload = {
            "skeletons": [
                {
                    "id": 1,
                    "joints": {
                        "RightShoulder": {"x": 0.3, "y": 0.0, "z": 0.0},
                        "RightElbow": {"x": 0.3, "y": 0.25, "z": 0.0},
                        "RightWrist": {
                            "x": 0.3,
                            "y": 0.5 + math.sin(t) * 0.2, 
                            "z": 0.0 + math.cos(t) * 0.2,
                            "confidence": 1.0
                        },
                        "LeftShoulder": {"x": -0.3, "y": 0.0, "z": 0.0},
                        "LeftElbow": {"x": -0.3, "y": 0.25, "z": 0.0},
                        "LeftWrist": {
                            "x": -0.3,
                            "y": 0.5,
                            "z": 0.0,
                            "confidence": 0.8
                        }
                    }
                }
            ]
        }
        
        message = json.dumps(payload).encode('utf-8')
        sock.sendto(message, (UDP_IP, UDP_PORT))
        print(f"Sent frame {frame}", end='\r')
        
        frame += 1
        time.sleep(1/30.0) # 30 Hz

except KeyboardInterrupt:
    print("\nStopped.")
