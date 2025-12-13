import socket
import time

# ⚠️ REMPLACE PAR L'IP DE TA CARTE STM32 (regarde les logs UART pour la trouver)
STM32_IP = "192.168.129.181" 
PORT = 12345

print(f"tentative de connexion à {STM32_IP}:{PORT}...")

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((STM32_IP, PORT))
    
    # On envoie la commande attendue par ton code C (strstr "data_request")
    message = '{ "type": "data_request" }'
    print(f"📤 Envoi : {message}")
    s.send(message.encode())
    
    # On attend la réponse
    response = s.recv(1024)
    print(f"📥 Réponse de la STM32 : {response.decode('utf-8')}")
    
    s.close()

except ConnectionRefusedError:
    print("❌ Connexion refusée. Le serveur TCP sur la STM32 ne tourne pas ou mauvaise IP.")
except Exception as e:
    print(f"❌ Erreur : {e}")