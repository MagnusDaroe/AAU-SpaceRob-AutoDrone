import socket
import socket
import time

HOST = "192.168.60.251"  # Replace 'ubuntu_ip' with the IP address of your Ubuntu machine
PORT = 12345        # The port on which the server is listening

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))
    print("Connected to server")

    while True:
        #data = input("Enter data to send (or 'exit' to quit): ")
        data="2\n"
        if data.lower() == 'exit':
            break
        client_socket.sendall(data.encode())

print("Connection closed")
