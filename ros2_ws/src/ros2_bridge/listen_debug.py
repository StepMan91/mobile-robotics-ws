import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"DEBUG LISTENER: Listening on {UDP_IP}:{UDP_PORT}")
print("Waiting for packets...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"RECEIVED MESSAGE from {addr}: {data}")
