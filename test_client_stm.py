import socket

IP = "0.0.0.0"   # écoute sur toutes les interfaces
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((IP, PORT))
sock.listen(5)

print(f"📡 Serveur Python en écoute sur le port {PORT}...")

while True:
    conn, addr = sock.accept()
    print("\n🔵 Connexion depuis :", addr)

    data = conn.recv(1024).decode()
    print("📩 Message reçu :", data)

    conn.close()
