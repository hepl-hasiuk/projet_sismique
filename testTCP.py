import socket

s = socket.socket()
s.connect(("192.168.1.181", 5000))

while True:
    data = s.recv(1024)
    if not data:
        break
    print(data.decode(), end="")
