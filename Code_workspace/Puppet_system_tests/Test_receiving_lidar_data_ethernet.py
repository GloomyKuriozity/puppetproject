import socket

def connect_to_lidar(ip, port, interface=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    
    if interface:
        s.setsockopt(socket.SOL_SOCKET, 25, str(interface + '\0').encode('utf-8'))

    try:
        print(f"Attempting to connect to LiDAR at {ip}:{port}")
        s.connect((ip, port))
        print(f"Connected to LiDAR at {ip}:{port}")
        return s
    except socket.error as e:
        print(f"Socket error: {e}")
        return None

def start_data_streaming(s):
    try:
        # Command to start data streaming
        command = b'\x02sEN LMDscandata 1\x03'
        s.sendall(command)
        print("Data streaming command sent.")
    except socket.error as e:
        print(f"Error sending data streaming command: {e}")

def read_lidar_data(s):
    buffer = b''
    try:
        while True:
            data = s.recv(4096)
            if data:
                buffer += data
                if buffer.endswith(b'\x03'):
                    distances = parse_lidar_data(buffer)
                    if distances:
                        angle_distance_pairs = associate_with_angles(distances, angular_resolution=0.5)
                        for angle, distance in angle_distance_pairs[:540]:  # Print the first 10 for example
                            print(f"Angle: {angle:.2f}°, Distance: {distance} mm")
                    buffer = b''
            else:
                break
    except socket.timeout:
        print("Error reading data: timed out")
    except socket.error as e:
        print(f"Error reading data: {e}")
    finally:
        s.close()

def parse_lidar_data(data):
    data = data.strip(b'\x02\x03').decode('ascii')
    
    # Split the data into individual components
    parts = data.split()
    
    try:
        # Locate the 'DIST1' header, which indicates the start of the distance data
        dist_index = parts.index('DIST1') + 7  # Skip 7 elements after 'DIST1' to reach the distance data
        distances = parts[dist_index:]
    except ValueError:
        print("DIST1 header not found in data")
        return []

    # Filter out non-hexadecimal components
    measurements = [comp for comp in distances if all(c in '0123456789ABCDEF' for c in comp)]

    # Convert hex measurements to decimal
    distances = [int(measurement, 16) for measurement in measurements]
    
    return distances

def associate_with_angles(distances, start_angle=0, angular_resolution=0.25):
    num_measurements = len(distances)
    angles = [start_angle + i * angular_resolution for i in range(num_measurements)]
    
    return list(zip(angles, distances))

if __name__ == "__main__":
    lidar_ip = "169.254.113.94"  # Updated LiDAR IP address
    lidar_port = 2111            # Updated port
    network_interface = "enth0"  # Your network interface name
    lidar_socket = connect_to_lidar(lidar_ip, lidar_port, network_interface)
    if lidar_socket:
        start_data_streaming(lidar_socket)
        read_lidar_data(lidar_socket)
