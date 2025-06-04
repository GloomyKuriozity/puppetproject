import socket
import os

# === CONFIGURATION ===
USER=os.getenv('USERNAME')  # Get the current username
MAPS_FOLDER = f"C:/home/{USER}/maps_library"  # Path to the maps folder
HOST = 'localhost'  # Hostname or IP address
PORT = 5001        # Different port than robot control

def handle_client(conn):
    print("Client connected.")

    try:
        # Step 1: Send the list of maps (yaml files only)
        yaml_files = [f for f in os.listdir(MAPS_FOLDER) if f.endswith('.yaml')]
        yaml_list = "\n".join(yaml_files) + "\nEND\n"
        conn.sendall(yaml_list.encode('utf-8'))

        # Step 2: Wait for map name request
        requested_file = conn.recv(1024).decode('utf-8').strip()
        print(f"Requested map: {requested_file}")

        yaml_path = os.path.join(MAPS_FOLDER, requested_file)
        pgm_path = yaml_path.replace('.yaml', '.pgm')

        for path in [yaml_path, pgm_path]:
            if os.path.exists(path):
                send_file(conn, path)
            else:
                print(f"File not found: {path}")

    except Exception as e:
        print(f"Error handling client: {e}")
    finally:
        conn.close()
        print("Connection closed.")

def send_file(conn, filepath):
    filesize = os.path.getsize(filepath)
    filename = os.path.basename(filepath)
    print(f"Sending {filename} ({filesize} bytes)")

    # Step 1: Send filename and size
    header = f"{filename}|{filesize}\n"
    conn.sendall(header.encode('utf-8'))

    # Step 2: Send file data
    with open(filepath, 'rb') as f:
        while True:
            data = f.read(4096)
            if not data:
                break
            conn.sendall(data)

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}...")

        while True:
            conn, addr = s.accept()
            handle_client(conn)

if __name__ == "__main__":
    main()
