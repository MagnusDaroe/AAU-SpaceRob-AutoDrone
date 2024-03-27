import socket

HOST = "172.18.64.1"  # Replace 'ubuntu_ip' with the IP address of your Ubuntu machine
PORT = 12345        # The port on which the server is listening

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))
    print("Connected to server")

    while True:
        data = input("Enter data to send (or 'exit' to quit): ")
        if data.lower() == 'exit':
            break
        client_socket.sendall(data.encode())

print("Connection closed")
