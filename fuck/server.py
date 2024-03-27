import socket

HOST = '172.18.64.1'  # Listen on all network interfaces
PORT = 12345      # Choose a port to listen on

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()
    print(f"Connection from {addr}")

    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"Received: {data.decode()}")
