import socket

HOST = '0.0.0.0' # Écoute partout
PORT = 12345     # Port 1234

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print(f"🏠 Serveur Python en attente de la STM32 sur le port {PORT}...")


while True:
    client_socket, addr = server_socket.accept()
    print(f"🟢 Connexion entrante de la STM32 : {addr}")
    
    try:
        data = client_socket.recv(1024)
        if data:
            print(f"📩 Reçu : {data.decode('utf-8')}")
            # ... (dans la boucle while True, après le print "Reçu") ...
        if data:
            print(f"📩 Reçu : {data.decode('utf-8')}")

            # 🚨 AJOUT DE LA RÉPONSE
            response_json = """{
                "type": "data_response",
                "id": "server-python",
                "timestamp": "2025-10-02T08:21:01Z",
                "acceleration": {"x": 0.05, "y": 0.02, "z": 0.98},
                "status": "normal"
            }"""
            
            client_socket.send(response_json.encode('utf-8'))
            print("📤 Réponse envoyée à la STM32")
            # Tu peux répondre si tu veux, mais ton code C ferme la connexion juste après l'envoi
    except Exception as e:
        print(f"Erreur lecture: {e}")
        
    client_socket.close()
    print("🔴 Connexion fermée.\n")