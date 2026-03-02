import socket

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's Algorithm
    server_socket.bind(('0.0.0.0', 5000))
    server_socket.listen(5)
    print("Listening on port 5000")

    try:
        while True:
            client_socket, addr = server_socket.accept()
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)  # Increase buffer size
            client_socket.settimeout(0.05)  # Reduce polling time
            print(f"Received connection from {addr}")

            try:
                while True:
                    try:
                        data = client_socket.recv(1024)
                        if data:
                            print(f"Received message: {data.decode('utf-8')}")
                        else:
                            print("No more data from the client.")
                            break
                    except socket.timeout:
                        continue  # No data, continue polling
            finally:
                client_socket.close()

    except KeyboardInterrupt:
        print("Server is shutting down.")
        server_socket.close()

if __name__=="__main__":
    main()
