  GNU nano 7.2                                                                                                                                                                                                                                                                                                                                                                                                                                                                      app_command.py                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
'''
Author: Mélanie Geulin

Last Update: 04/12/2025

Script: app_command

Summary: Manages data between wifi telecommand commands and robot movement and actions
'''

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray, Float32
import threading
import select
import time
import subprocess
import os
import gpiod
from datetime import datetime

CMD_VEL_TIMEOUT = 1.0  # Timeout after which the robot stops if no command is received
CMD_VEL_MIN_CHANGE = 0.05  # Minimum change in linear/angular values to publish
HEARTBEAT_INTERVAL = 1.0  # Interval in seconds to expect/receive heartbeat

GPIO_CHIP = "gpiochip4"
GPIOSET_CMD = "/usr/bin/gpioset"  # adjust if `which gpioset` says something else

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
        self.contamination_sub = self.create_subscription(
            Float32MultiArray,
            'sfp100/contamination',          # change if your topic is named differently
            self.on_contamination,
            10
        )

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

        # GPIO / probe control state (we use gpioset on gpiochip4)
        self.start_time = time.time()
        self.PROBE_ARM_DELAY_S = 2.0
        self.gpio_up_pin = 24
        self.gpio_down_pin = 25
        self.home_active = False
        self.gpio_lock = threading.Lock()
        # Request both GPIO lines once and keep ownership (prevents float/glitches)
        self.gpio_chip = gpiod.Chip(GPIO_CHIP)
        self.gpio_up_line = self.gpio_chip.get_line(self.gpio_up_pin)
        self.gpio_down_line = self.gpio_chip.get_line(self.gpio_down_pin)

        self.gpio_up_line.request(consumer="app_command", type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.gpio_down_line.request(consumer="app_command", type=gpiod.LINE_REQ_DIR_OUT, default_val=0)

        # Force safe state at startup
        self._set_gpio_state(up=False, down=False)

        self.publish_log(
            f"GPIO control using gpioset on {GPIO_CHIP} "
            f"(UP={self.gpio_up_pin}, DOWN={self.gpio_down_pin})"
        )


        # Separate thread for socket communication
        self.socket_thread = threading.Thread(target=self.socket_worker, daemon=True)
        self.socket_thread.start()

        # ROS timer to check for command timeout
        self.HEARTBEAT_MESSAGE = "0 0 0 0 0 0\n"  # Message to send as heartbeat "InMissionState, N/A, N/A, N/A, N/A, N/A"
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
    def on_contamination(self, msg: Float32MultiArray):
        """
        Forward contamination value to the tablet over TCP.
        msg.data = [alpha_cps, alpha_bq_cm2, beta_gamma_cps, beta_gamma_bq_cm2]
        We choose beta_gamma_bq_cm2 as the value to send.
        """
        if not msg.data:
            return
    
        try:
            alpha_cps, alpha_bq_cm2, beta_gamma_cps, beta_gamma_bq_cm2 = msg.data
        except ValueError:
            # Unexpected length, just log and skip
            self.get_logger().warn(f"Unexpected contamination array length: {len(msg.data)}")
            return
    
        value = beta_gamma_bq_cm2   # pick what you want to display in the app
    
        line = f"DATA:CONTAM:{value:.6f}\n"
    
        if self.connected and self.client_socket:
            try:
                self.client_socket.sendall(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error sending contamination data: {e}")
                self.disconnect_client()
    
    def _set_gpio_state(self, up: bool, down: bool):
        """Set GPIO 24 (UP) and 25 (DOWN) using persistent libgpiod lines."""
        with self.gpio_lock:
            if up and down:
                self.publish_log("Requested UP and DOWN at the same time, forcing both LOW.")
                up = False
                down = False

            self.publish_log(f"GPIO request: UP={up}, DOWN={down}")

            try:
                # Set both outputs. (Not truly atomic in libgpiod v1, but stable and non-floating.)
                self.gpio_up_line.set_value(1 if up else 0)
                self.gpio_down_line.set_value(1 if down else 0)
            except Exception as e:
                self.publish_log(f"Error setting GPIO via gpiod: {e}")

    def handle_up_command(self):
        """Handle 'UP' command from tablet: UP=HIGH, DOWN=LOW, cancel HOME if active."""
        if time.time() - self.start_time < self.PROBE_ARM_DELAY_S:
            self.publish_log("Probe command ignored during arming delay.")
            return

        if self.home_active:
            self.publish_log("UP command received while HOME is active, ignoring.")
            return

        self.publish_log("Handling UP command (GPIO24 HIGH, GPIO25 LOW).")
        self._set_gpio_state(up=True, down=False)

    def handle_down_command(self):
        """Handle 'DOWN' command from tablet: DOWN=HIGH, UP=LOW, cancel HOME if active."""
        if time.time() - self.start_time < self.PROBE_ARM_DELAY_S:
            self.publish_log("Probe command ignored during arming delay.")
            return

        if self.home_active:
            self.publish_log("DOWN command received while HOME is active, ignoring.")
            return

        self.publish_log("Handling DOWN command (GPIO25 HIGH, GPIO24 LOW).")
        self._set_gpio_state(up=False, down=True)

    def handle_home_command(self):
        """
        Handle 'HOME' command: maintain UP movement for 8s, then stop both.
        During HOME, ignore manual UP/DOWN commands.
        """
        if time.time() - self.start_time < self.PROBE_ARM_DELAY_S:
            self.publish_log("Probe command ignored during arming delay.")
            return

        if self.home_active:
            self.publish_log("HOME already in progress, ignoring new HOME command.")
            return

        self.publish_log("Handling HOME command: UP for 8 seconds, then stop.")
        self.home_active = True
    
        def home_sequence():
            # Start UP
            self._set_gpio_state(up=True, down=False)
            time.sleep(8.0)
            # Stop
            self._set_gpio_state(up=False, down=False)
            self.home_active = False
            self.publish_log("HOME sequence completed, GPIO UP/DOWN set to LOW.")

        threading.Thread(target=home_sequence, daemon=True).start()

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
                self._set_gpio_state(up=False, down=False)
                self.send_stop_command()
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
            case "UP":
                self.handle_up_command()
            case "DOWN":
                self.handle_down_command()
            case "HOME":
                self.handle_home_command()
            case "STOP_PROBE":
                self.handle_probe_stop_command()  # <-- add this
            case _ if ";" in data:
                self.process_twist_command(data)
            case _:
                self.publish_log(f"Unknown command: {data}")

    def handle_probe_stop_command(self):
        """
        Stop probe motion immediately: both UP and DOWN GPIO low.
        Also cancels HOME if it was running.
        """
        if self.home_active:
            self.publish_log("STOP_PROBE received: cancelling HOME and stopping probe.")
            self.home_active = False
        else:
            self.publish_log("STOP_PROBE received: stopping probe (UP/DOWN LOW).")

        self._set_gpio_state(up=False, down=False)


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

        try:
            self._set_gpio_state(up=False, down=False)
        except Exception:
            pass

        try:
            if hasattr(self, "gpio_up_line"):
                self.gpio_up_line.release()
            if hasattr(self, "gpio_down_line"):
                self.gpio_down_line.release()
            if hasattr(self, "gpio_chip"):
                self.gpio_chip.close()
        except Exception as e:
            self.get_logger().error(f"Error releasing GPIO: {e}")

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
