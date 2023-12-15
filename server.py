import socket
import json

def load_config():
    with open("config.json", "r") as file:
        config = json.load(file)
    return config

def start_udp_server(host, port):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((host, port))

    print(f"Servidor UDP iniciado en {host}:{port}")

    while True:
        data, addr = udp_socket.recvfrom(1024)  # Tamaño del buffer
        formatted_data = data.decode('utf-8')
        print(f"Mensaje recibido de {addr}:\n\n{formatted_data}")

if __name__ == "__main__":
    config = load_config()
    server_host = config["host"]
    server_port = config["port"]

    start_udp_server(server_host, server_port)
