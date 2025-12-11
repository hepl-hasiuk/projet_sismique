import socket

STM32_IP = "192.168.1.181"   # ⚠️ MET TON IP DE CARTE ICI
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((STM32_IP, PORT))

msg = '{ "type": "testblbla" }'
sock.send(msg.encode())

data = sock.recv(1024).decode()
print("\n📩 Réponse reçue :", data)

sock.close()
