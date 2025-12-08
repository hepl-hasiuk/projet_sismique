#  Desactiver pare feu reseau public 

import sys
print("PYTHON EXE =", sys.executable)

import socket
import json

def udp_server(ip="0.0.0.0", port=1234):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"Serveur UDP en écoute sur {ip}:{port}")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"\n📡 Message reçu de {addr[0]} :")

        try:
            message = json.loads(data.decode())
            print(message)
        except:
            print("Données :", data.decode())

udp_server() 
# 