import socket
import time

HOST = '192.168.60.251'  # Listen on all network interfaces
PORT = 12345      # Choose a port to listen on

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()
    print(f"Connection from {addr}")

    prev_time = time.time()  # Initialize previous time

    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"Received: {data.decode()}")

            current_time = time.time()  # Get current time
            time_diff = current_time - prev_time  # Calculate time difference
            frequency = 1 / time_diff  # Calculate frequency (Hz)
            prev_time = current_time  # Update previous time

            print(f"Frequency: {frequency} Hz")
