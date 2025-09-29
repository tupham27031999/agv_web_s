import socket

UDP_PORT = 2368
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
print(f"Listening on UDP {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(65535)
    ip_src = addr[0]
    if ip_src == "192.168.1.50":
        print("LiDAR1:", len(data), "bytes")
    elif ip_src == "192.168.1.51":
        print("LiDAR2:", len(data), "bytes")
    else:
        print("Unknown source:", addr, len(data))
