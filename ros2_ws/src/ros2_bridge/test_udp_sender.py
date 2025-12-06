import socket
import json
import time
import math

UDP_IP = "127.0.0.1"
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
                        "RightWrist": {
                            "x": 0.3,
                            "y": 0.5 + math.sin(t) * 0.2, # Moving up/down
                            "z": 1.0 + math.cos(t) * 0.2, # Moving fwd/back
                            "confidence": 1.0
                        },
                        "LeftWrist": {
                            "x": -0.3,
                            "y": 0.5,
                            "z": 1.0,
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
