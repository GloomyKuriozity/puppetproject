'''
Author: Mélanie Geulin

Last Update: 23/04/2025

Script: scan_from_lidar

Summary: Process LIDAR data from physical to LaserScan message in /scan topic
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import numpy as np
import time
import signal
import sys
import math
from scipy.signal import medfilt
from rclpy.qos import QoSProfile


MAX_BUFFER_SIZE = 100000
MAX_BACKOFF_TIME = 32
TIMER_PERIOD = 0.05  # 1 / 15
SOCKET_TIMEOUT = 0.5
MAX_RETRY_TIME = 5
READ_TIMEOUT = 2

class LidarPublisher(Node):
#####INITIALIZATION#####
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', qos_profile=QoSProfile(depth=100))

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.reconnecting = False
        self.reading_data = False

        self.lidar_ip = "169.254.113.94"
        self.lidar_port = 2111
        self.network_interface = "eth0"
        self.lidar_socket = None

        self.buffer = b''
        self.connect_to_lidar(self.lidar_ip,self.lidar_port,self.network_interface)
########################

#####CALLBACKS#####
    def timer_callback(self):
        '''
        Callbacks 10Hz every messages from LIDAR
        '''
        if self.lidar_socket:
            self.process_lidar_data()
        elif not self.reconnecting:
            self.get_logger().info("Attempting to reconnect...")
            self.reconnecting = True
            self.connect_to_lidar(self.lidar_ip,self.lidar_port,self.network_interface)
###################

#####FUNCTIONS#####
    def connect_to_lidar(self, ip, port, interface=None, retries=3):
        '''
        Connection to physical LIDAR Tim571 SICK
        Adjust received buffer size size depending on test performance
        '''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(SOCKET_TIMEOUT)

        if interface:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)

        backoff = 1  # Initial backoff time (in seconds)

        for attempt in range(retries):
            try:
                self.get_logger().info(f"Attempt {attempt+1}: Connecting to LiDAR at {self.lidar_ip}:{self.lidar_port}")
                s.connect((self.lidar_ip, self.lidar_port))
                s.setblocking(False)
                self.get_logger().info(f"Connected to LiDAR at {self.lidar_ip}:{self.lidar_port}")
                self.start_data_streaming(s)
                self.lidar_socket = s  # Update lidar_socket
                self.reconnecting = False
                return
            except socket.timeout:
                self.get_logger().warning(f"Connection timed out. Retrying in {backoff} seconds...")
                time.sleep(backoff)
                backoff = min(backoff * 2, MAX_BACKOFF_TIME)
            except socket.error as e:
                self.get_logger().error(f"Socket error: {e}. Retrying in {backoff} seconds...")
                time.sleep(backoff)
                backoff = min(backoff * 2, MAX_BACKOFF_TIME)
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}. Retrying in {backoff} seconds...")
                time.sleep(backoff)

        self.get_logger().warning(f"Failed to connect after {retries} attempts. System will retry in the next cycle.")
        self.reconnecting = False

    def read_lidar_data(self, s):
        '''
        Reading data ranges from physical LIDAR
        WARN: Change asyncio.TimeoutError timeout value if not sufficient for IRL latency.
        '''
        try:
            data = self.lidar_socket.recv(4096)
            if data:
                self.buffer += data
                if len(self.buffer) > MAX_BUFFER_SIZE:
                    self.get_logger().warning(f"Buffer exceeded maximum size ({len(self.buffer)} bytes). Resetting buffer.")
                    self.buffer = b''

                while b'\x03' in self.buffer:
                    frame, self.buffer = self.buffer.split(b'\x03', 1)
                    frame += b'\x03'
                    distances, start_angle, angular_res = self.parse_lidar_data(frame)
                    if distances:
                        return distances, start_angle, angular_res

        except socket.timeout:
            self.get_logger().error("Socket timeout while reading LiDAR data.")
        except socket.error as e:
            self.get_logger().error(f"Error reading data: {e}.")
            if self.lidar_socket:
                self.lidar_socket.close()  # Ensure socket is closed
                self.lidar_socket = None  # Reset to None for reconnection
            if not self.reconnecting:
                self.reconnecting = True
                self.connect_to_lidar(self.lidar_ip, self.lidar_port, self.network_interface)
        except Exception as e:
            self.get_logger().error(f"Unexpected error during reading: {e}")

        self.get_logger().warning(f"Reading LiDAR data timed out after {READ_TIMEOUT} seconds.")
        return [], -2.35619, 0.00575958653

    def smooth_ranges(self, ranges, kernel_size=5):
        """
        Apply a moving average filter to smooth LiDAR ranges.
        Args:
            ranges: List of LiDAR range values.
            kernel_size: The size of the moving window.
        Returns:
            Smoothed range values.
        """
        smoothed_ranges = medfilt(ranges)
        return smoothed_ranges.tolist()

    def process_lidar_data(self):
        '''
        Process ranges received by physical LIDAR
        '''
        if self.lidar_socket and not self.reading_data:
            start_time = self.get_clock().now() # Record start time
            self.reading_data = True  # Indicate that we are in the middle of a read
            distances, start_angle, angular_res = self.read_lidar_data(self.lidar_socket)

            if distances:
                angle_distance_pairs = self.associate_with_angles(distances, start_angle, angular_res)
                scan = LaserScan()
                scan.header.stamp = start_time.to_msg()
                scan.header.frame_id = 'base_scan'

                num_ranges = len(distances)
                scan.angle_min = start_angle
                scan.angle_max = start_angle + angular_res * (len(distances) - 1)
                scan.angle_increment = angular_res
                scan.time_increment = 0.1 / len(distances)  # Time between measurements
                scan.scan_time = 0.1  # Total scan time
                scan.range_min = 0.05  # Minimum range
                scan.range_max = 25.0  # Maximum range

                # Convert distances to meters
                scan.ranges = [
                    float(distance) / 1000.0 if distance > 0 else float('inf') 
                    for _, distance in angle_distance_pairs
                ]

                # Before publishing
                scan.ranges = self.smooth_ranges(scan.ranges,  kernel_size=5)

                # Assuming no intensity data available
                scan.intensities = []
                self.publisher_.publish(scan)

                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e6
                self.get_logger().info(f"Published scan with {len(scan.ranges)} points. Processing took {elapsed_time:.4f} seconds.")

            else:
                # If reading data failed, try to reconnect
                self.get_logger().error("Failed to read LiDAR data, attempting to reconnect.")
                self.lidar_socket = None
                if not self.reconnecting:
                    self.reconnecting = True
                    self.connect_to_lidar(self.lidar_ip,self.lidar_port,self.network_interface)

            self.reading_data = False  # Reset the flag after processing
################################

#####OTHER FUNCTIONS#####
    def start_data_streaming(self, s):
        '''
        Start publication of data from ethernet connection to PC
        '''
        try:
            command = b'\x02sEN LMDscandata 1\x03'
            s.sendall(command)
            self.get_logger().info("Data streaming command sent.")
        except socket.error as e:
            self.get_logger().error(f"Error sending data streaming command: {e}")

    def parse_lidar_data(self, data):
        '''
        Decode data received from physical LIDAR
        '''
        try:
            data = data.strip(b'\x02\x03').decode('ascii')
            parts = data.split()

            # Extract distances
            dist_index = parts.index('DIST1') + 7
            distance_data = parts[dist_index:]
            measurements = [comp for comp in distance_data if all(c in '0123456789ABCDEF' for c in comp)]
            distances = [int(m, 16) for m in measurements]

            # Extract start angle (1/10000°) and angular step (1/10000°)
            # These fields are fixed index positions in the telegram (based on SICK format)
            # Start angle is at index 23, angular step is at 24 (adjust if needed based on your telegram format)
            start_angle_raw = parts[23]
            angular_step_raw = parts[24]
            start_angle = int(start_angle_raw, 16) / 10000.0  # now in degrees
            angular_step = int(angular_step_raw, 16) / 10000.0  # in degrees

            # Convert to radians
            start_angle_rad = math.radians(start_angle)
            angular_step_rad = math.radians(angular_step)

            return distances, start_angle_rad, angular_step_rad

        except Exception as e:
            self.get_logger().error(f"Failed to parse LIDAR data: {e}")
            return [], -2.35619, 0.00575958653  # fallback: -135° and 0.33° in radians

    def associate_with_angles(self, distances, start_angle, angular_resolution):
        num_measurements = len(distances)
        angles = np.linspace(start_angle, start_angle + angular_resolution * (num_measurements - 1), num_measurements)
        return list(zip(angles, distances))

#########################

#####MAIN#####
def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()

    # Register signal handler for SIGINT and SIGTERM
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, lidar_publisher))
    signal.signal(signal.SIGTERM, lambda sig, frame: signal_handler(sig, frame, lidar_publisher))

    try:
        rclpy.spin(lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_publisher.get_logger().info("Shutting down node...")
        lidar_publisher.destroy_node()
        rclpy.shutdown()
        lidar_publisher.get_logger().info("Node has been shut down successfully.")

def signal_handler(sig, frame, node):
    node.get_logger().info("Received signal, shutting down...")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
