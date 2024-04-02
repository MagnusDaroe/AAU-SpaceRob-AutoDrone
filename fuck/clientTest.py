import socket
import time

HOST = "192.168.60.251"  # Replace 'ubuntu_ip' with the IP address of your Ubuntu machine
PORT = 12345        # The port on which the server is listening

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    print("Connected to server")
    client_socket.connect((HOST, PORT))

    while True:
        #data = input("Enter data to send (or 'exit' to quit): ")
        time.sleep(0.01)
        data="2123213,212132321,312312,132213,132132,132321"
        
        if data.lower() == 'exit':
            break
        client_socket.sendall(data.encode())
            

print("Connection closed")