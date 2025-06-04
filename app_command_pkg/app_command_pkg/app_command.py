'''
Author: Mélanie Geulin

Last Update: 15/10/2024

Script: app_command

Summary: Manages data between wifi telecommand commands and robot movement and actions
'''

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import select
import time
import subprocess
import os
from datetime import datetime

CMD_VEL_TIMEOUT = 1.0  # Timeout after which the robot stops if no command is received
CMD_VEL_MIN_CHANGE = 0.05  # Minimum change in linear/angular values to publish
HEARTBEAT_INTERVAL = 1.0  # Interval in seconds to expect/receive heartbeat

user = os.getenv("USER")
filename = datetime.now().strftime("map_%Y%m%d_%H%M%S")
path = f"/home/{user}/ros2_ws/maps_library/{filename}"

class AppCommandNode(Node):
#####INITIALIZATION######
    def __init__(self):
        super().__init__('app_command')

        # Socket setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Low-latency

        try:
            self.server_socket.bind(('0.0.0.0', 5000))
        except socket.error as e:
            self.get_logger().error(f"Failed to bind socket: {e}")
            self.shutdown_flag = True

        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)  # Timeout for accept loop
        self.get_logger().info("Listening on port 5000")

        # ROS publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pause_request_publisher = self.create_publisher(String, 'pause_request', 10)
        self.log_publisher = self.create_publisher(String, 'app_command', 10)

        # Initialize connection variables
        self.teleoperation_active = False
        self.connected = False
        self.client_socket = None
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_heartbeat_time = self.get_clock().now()
        self.cmd_vel_lock = threading.Lock()

        # Store last published velocity to prevent spam
        self.last_published_cmd_vel = (0.0, 0.0)  # (linear, angular)
        self.shutdown_flag = False  # Flag to signal shutdown

        # Separate thread for socket communication
        self.socket_thread = threading.Thread(target=self.socket_worker, daemon=True)
        self.socket_thread.start()

        # ROS timer to check for command timeout
        self.HEARTBEAT_MESSAGE = "0 0 0 0 0 0"  # Message to send as heartbeat "InMissionState, N/A, N/A, N/A, N/A, N/A"
        self.heartbeat_timer = self.create_timer(HEARTBEAT_INTERVAL, self.send_heartbeat)
#########################

#####CALLBACKS######
    def send_heartbeat(self):
        """Send a heartbeat message to the client to indicate the server (robot) is alive and which state."""
        if self.connected and self.client_socket:
            try:
                self.client_socket.sendall(self.HEARTBEAT_MESSAGE.encode('utf-8'))
                self.get_logger().info("Sent heartbeat to client.")
            except socket.error as e:
                self.get_logger().error(f"Error sending heartbeat: {e}")
               # self.disconnect_client()
####################

#####OTHER FUNCTIONS#####
    def socket_worker(self):
        """Separate thread to handle socket connections and data processing."""
        while rclpy.ok() and not self.shutdown_flag:
            if not self.connected:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
                    self.client_socket.settimeout(1.0)
                    self.publish_log(f"Connected to {addr}")
                    self.connected = True
                except socket.timeout:
                    pass  # Try accepting again if no connection yet
                except socket.error as e:
                    self.get_logger().error(f"Socket error while accepting connection: {e}")

            if self.connected:
                try:
                    ready = select.select([self.client_socket], [], [], 0.1)
                    if ready[0]:
                        start_time = time.time()  # Start latency monitoring
                        data = self.client_socket.recv(1024)
                        if data:
                            self.process_data(data.decode('utf-8').strip())
                        else:
                            self.publish_log("Client disconnected.")
                            self.disconnect_client()
                        latency = time.time() - start_time
                        self.get_logger().info(f"Socket message processed in {latency:.4f} seconds")
                    else:
                        continue
                except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                    self.publish_log("Client disconnected.")
                    self.disconnect_client()
                except socket.error as e:
                    self.get_logger().error(f"Socket error during data read: {e}")
                    self.disconnect_client()
                except Exception as e:
                    self.get_logger().error(f"Unexpected error processing socket data: {e}")
                    self.disconnect_client()

    def disconnect_client(self):
        """Handles proper disconnection of the client."""
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
                self.publish_log("Client socket closed.")
            except Exception as e:
                self.get_logger().error(f"Error closing client socket: {e}")
        self.client_socket = None
        self.connected = False  # Properly reset the connected flag

    def process_data(self, data):
        """Process incoming data, match known commands, and log actions."""
        self.publish_log(f"Processing data: {data}")
        match data:
            case "Start Exploration":
                self.start_exploration()
            case "Stop Exploration":
                self.stop_exploration()
            case "Pause Robot":
                self.send_pause_command()
            case "Continue Exploration":
                self.send_continue_command()
            case _ if ";" in data:
                self.process_twist_command(data)
            case _:
                self.publish_log(f"Unknown command: {data}")

    def send_pause_command(self):
        """Send a pause command to the robot."""
        msg = String()
        msg.data = "Pause"
        self.pause_request_publisher.publish(msg)
        self.publish_log("Published Pause command")

    def send_continue_command(self):
        """Send a continue command to the robot."""
        msg = String()
        msg.data = "Continue"
        self.pause_request_publisher.publish(msg)
        self.publish_log("Published Continue command")

    def start_exploration(self):
        """Handles starting the exploration process."""
        self.publish_log("Starting exploration process.")

        try:
            # Start the robot control node
            self.robot_control_process = subprocess.Popen(
                ['ros2', 'launch', 'puppet_irl_bringup','mapping.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.send_stop_command()
            self.publish_log("Started flipper_exploration.")

            self.HEARTBEAT_MESSAGE = "1 0 0 0 0 0"
            self.publish_log("Started cartographer.launch.py.")

        except Exception as e:
            self.publish_log(f"Error starting exploration: {str(e)}")

    def stop_exploration(self):
        """Handles stopping the exploration process."""
        self.publish_log("Stopping exploration process.")

        # Stop exploration nodes using 'killall'
        try:
            # Kill the naviguation and exploration nodes
            subprocess.run(['pkill', 'f', 'explore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            subprocess.run(['pkill', 'f', 'nav2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            self.publish_log("Successfully killed slam node, naviguation node and exploration node.")

        except Exception as e:
            self.publish_log(f"Error stopping exploration nodes: {str(e)}")

        #Save map made by slam
        try:
            time.sleep(2)  # Wait a bit before saving the map
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            self.publish_log("Successfully saved map to maps_library folder.")

        except Exception as e:
            self.publish_log(f"Error saving slam map: {str(e)}")

        # Stop slam:
        try:
            subprocess.run(['pkill', 'f', 'async_slam_tool'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            self.publish_log("Successfully killed SLAM node.")

        except Exception as e:
            self.publish_log(f"Error stopping slam node: {str(e)}")

    def process_twist_command(self, data):
        """Processes and publishes velocity commands from the client."""
        try:
            linear, angular = map(str.strip, data.split(';'))
            linear = float(linear.replace(',', '.'))
            angular = float(angular.replace(',', '.'))

            self.teleoperation_active = True

            with self.cmd_vel_lock:
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                self.cmd_vel_publisher.publish(twist)
                self.last_cmd_vel_time = self.get_clock().now()
                self.last_published_cmd_vel = (linear, angular)  # Update the last published values
                self.publish_log(f"Published cmd_vel - Linear: {linear}, Angular: {angular}")

        except ValueError:
            self.publish_log(f"Invalid twist command format: {data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in processing twist command: {e}")

    def send_stop_command(self):
        """Send a stop command to the robot by setting velocities to zero."""
        with self.cmd_vel_lock:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.publish_log("Published stop command")

    def publish_log(self, message):
        """Publish a log message."""
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
        self.get_logger().info(message)

    def shutdown(self):
        """Gracefully shutdown socket and threads."""
        self.shutdown_flag = True
        self.disconnect_client()
        if self.server_socket:
            try:
                self.server_socket.shutdown(socket.SHUT_RDWR)
                self.server_socket.close()
                self.publish_log("Server socket closed.")
            except Exception as e:
                self.get_logger().error(f"Error closing server socket: {e}")
#########################

####MAIN#####
def main(args=None):
    rclpy.init(args=args)
    node = AppCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()  # Call shutdown on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#############
