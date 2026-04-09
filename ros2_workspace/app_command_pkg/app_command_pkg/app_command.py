'''
Author: Melanie Geulin

Mail: melanie.geulin@orano.group

Last Update: 09/04/2026

Script: app_command

Summary: Manages data between wifi telecommand commands and robot actions
'''

import socket
import rclpy
import threading
import select
import time
import subprocess
import os
import gpiod
import math
import json
import base64
import array
from datetime import datetime
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

HEARTBEAT_INTERVAL = 1.0            # Interval in seconds to expect/receive heartbeat

GPIO_CHIP = "gpiochip4"             # gpiochip retreived directly from RP5 info
GPIOSET_CMD = "/usr/bin/gpioset"    # gpioset custom file in RP5

### ROBOT STATES ###
ROBOT_WAITING = "WAITING"
ROBOT_WORKING = "WORKING"
ROBOT_PAUSED = "PAUSED"
ROBOT_MAPPING = "MAPPING"
ROBOT_CHARGING = "CHARGING"
ROBOT_ERROR = "ERROR"
####################

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
        self.server_socket.settimeout(0.1)
        self.get_logger().info("Listening on port 5000")

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', cmd_vel_qos)
        self.pause_request_publisher = self.create_publisher(String, 'pause_request', 10)
        self.log_publisher = self.create_publisher(String, 'app_command', 10)
        
        # Subscribers
        self.contamination_sub = self.create_subscription(
            Float32MultiArray,
            'sfp100/contamination',      
            self.on_contamination,
            10
        )
        self.robot_info_sub = self.create_subscription(
            String,
            'robot_info',
            self.on_robot_info,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_map,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.on_pose,
            10
        )

        # Map info variables
        self.last_map_sent_time = 0.0
        self.latest_map_msg = None
        self.map_send_period = 2.0  
        self.mapping_process = None
        self.current_mapping_job = None
        self.home_dir = os.path.expanduser("~")
        self.jobs_root = os.path.join(self.home_dir, "ros2_ws", "mapping_jobs")
        os.makedirs(self.jobs_root, exist_ok=True)

        #Pose info variables
        self.latest_pose_x = None
        self.latest_pose_y = None
        self.latest_pose_yaw = None
        self.last_pose_sent_time = 0.0
        self.pose_send_period = 0.2

        # Battery info variables
        self.latest_battery_voltage = None
        self.latest_battery_percentage = None
        self.latest_battery_low_warning = 0
        self.latest_battery_critical_warning = 0

        # Connection info variables
        self.teleoperation_active = False
        self.connected = False
        self.client_socket = None
        self.rx_buffer = ""
        self.last_heartbeat_time = self.get_clock().now() 
        self.disconnect_time = None
        self.safety_timer = self.create_timer(1.0, self.safety_watchdog_callback)
        self.last_client_heartbeat_time = time.time()

        # Store last published velocity to prevent spam
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_lock = threading.Lock()
        self.last_published_cmd_vel = (0.0, 0.0)
        self.shutdown_flag = False

        # Contamination info variables
        self.data_stream_enabled = False

        # GPIO / probe control info variables
        self.start_time = time.time()
        self.PROBE_ARM_DELAY_S = 2.0
        self.gpio_up_pin = 24
        self.gpio_down_pin = 25
        self.home_active = False
        self.gpio_lock = threading.Lock()
        self.gpio_chip = gpiod.Chip(GPIO_CHIP)
        self.gpio_up_line = self.gpio_chip.get_line(self.gpio_up_pin)
        self.gpio_down_line = self.gpio_chip.get_line(self.gpio_down_pin)
        self.gpio_up_line.request(consumer="app_command", type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.gpio_down_line.request(consumer="app_command", type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self._set_gpio_state(up=False, down=False)
        self.publish_log(
            f"GPIO control using gpioset on {GPIO_CHIP} "
            f"(UP={self.gpio_up_pin}, DOWN={self.gpio_down_pin})"
        )

        # Separate thread for socket communication
        self.socket_thread = threading.Thread(target=self.socket_worker, daemon=True)
        self.socket_thread.start()

        # ROS timer to check for command timeout
        self.robot_state = ROBOT_WAITING
        self.heartbeat_timer = self.create_timer(HEARTBEAT_INTERVAL, self.send_heartbeat)
        self.teleop_keepalive_timer = self.create_timer(0.2, self.teleop_keepalive_callback)
#########################

#####CALLBACKS######
    def send_heartbeat(self):
        """
        Send a periodic heartbeat message to the connected client.

        Includes:
        - robot state
        - battery voltage
        - battery percentage
        - warning flags

        Used by the app to monitor connection health and robot status.
        """
        if self.connected and self.client_socket:
            try:
                voltage_str = "unknown" if self.latest_battery_voltage is None else f"{self.latest_battery_voltage:.2f}"
                percentage_str = "unknown" if self.latest_battery_percentage is None else f"{self.latest_battery_percentage:.1f}"

                heartbeat_message = (
                    f"HEARTBEAT:"
                    f"state={self.robot_state};"
                    f"battery_voltage={voltage_str};"
                    f"battery_percentage={percentage_str};"
                    f"low_warning={self.latest_battery_low_warning};"
                    f"critical_warning={self.latest_battery_critical_warning}\n"
                )

                self.client_socket.sendall(heartbeat_message.encode('utf-8'))
                self.get_logger().info(f"Sent heartbeat to client: {heartbeat_message.strip()}")

            except socket.error as e:
                self.get_logger().error(f"Error sending heartbeat: {e}")
                self.send_stop_command()
                self.disconnect_client()

    def on_robot_info(self, msg: String):
        """
        Parse and cache robot telemetry received from the 'robot_info' topic.

        Expected format:
        battery_voltage=XX; battery_percentage=XX; low_warning=0/1; critical_warning=0/1
        """
        try:
            fields = {}
            for item in msg.data.split(';'):
                if '=' in item:
                    key, value = item.split('=', 1)
                    fields[key.strip()] = value.strip()

            voltage_str = fields.get("battery_voltage", "unknown")
            percentage_str = fields.get("battery_percentage", "unknown")
            low_warning_str = fields.get("low_warning", "0")
            critical_warning_str = fields.get("critical_warning", "0")

            self.latest_battery_voltage = None if voltage_str == "unknown" else float(voltage_str)
            self.latest_battery_percentage = None if percentage_str == "unknown" else float(percentage_str)
            self.latest_battery_low_warning = int(low_warning_str)
            self.latest_battery_critical_warning = int(critical_warning_str)

        except Exception as e:
            self.get_logger().error(f"Error processing robot_info: {e}")               

    def on_contamination(self, msg: Float32MultiArray):
        """
        Receive contamination sensor data and forward it to the client if streaming is enabled.

        Expected data:
        [alpha_cps, beta_gamma_cps]

        Formatted and sent as:
        DATA:CONTAM:<alpha>;<beta>\n
        """
        if not msg.data:
            return

        try:
            alpha_cps, beta_gamma_cps = msg.data
        except ValueError:
            self.get_logger().warn(f"Unexpected contamination array length: {len(msg.data)}")
            return

        line = (
            f"DATA:CONTAM:"
            f"{alpha_cps:.6f};"
            f"{beta_gamma_cps:.6f}\n"
        )

        if self.connected and self.client_socket and self.data_stream_enabled:
            try:
                self.client_socket.sendall(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error sending contamination data: {e}")
                self.disconnect_client()

    def safety_watchdog_callback(self):
        """
        Safety watchdog executed periodically.

        Responsibilities:
        - Stop robot if client heartbeat is lost (safety-critical)
        - Detect stale mapping states
        - Automatically discard mapping session if client is disconnected too long
        """
        try:
            if self.connected and self.robot_state == ROBOT_MAPPING:
                elapsed_since_client_heartbeat = time.time() - self.last_client_heartbeat_time
                if elapsed_since_client_heartbeat >= 3.0:
                    self.publish_log("Client heartbeat timeout during mapping. Sending immediate stop command.")
                    self.send_stop_command()
                    self.disconnect_client()
                    return
        
            if self.robot_state != ROBOT_MAPPING:
                return

            # Case 1: mapping state is stale but process is already gone
            if self.mapping_process is None:
                self.publish_log("Detected stale MAPPING state with no active mapping process. Resetting to WAITING.")
                if self.current_mapping_job is not None:
                    self.current_mapping_job["status"] = "DISCARDED"
                    self.save_current_job_metadata()
                    self.current_mapping_job = None
                self.robot_state = ROBOT_WAITING
                self.disconnect_time = None
                return

            # Case 2: disconnected too long during mapping
            if not self.connected and self.disconnect_time is not None:
                elapsed = time.time() - self.disconnect_time
                if elapsed >= 10.0:
                    self.publish_log("Client lost for >10s during mapping. Discarding mapping session.")
                    self.stop_mapping_discard()
                    self.disconnect_time = None

        except Exception as e:
            self.publish_log(f"Safety watchdog error: {e}")

    def on_map(self, msg: OccupancyGrid):
        """
        Receive OccupancyGrid map data from SLAM and forward it to the client.

        - Throttles transmission to reduce bandwidth
        - Encodes raw map data as base64
        - Stores latest map for later saving (MAP_SAVE_FINAL)

        Format sent:
        DATA:MAP:<width>;<height>;<resolution>;<origin_x>;<origin_y>;<data>
        """
        if not self.connected or self.client_socket is None:
            return

        try:
            now = time.time()

            if now - self.last_map_sent_time < self.map_send_period:
                return
            self.last_map_sent_time = now
            self.latest_map_msg = msg

            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            if width <= 0 or height <= 0 or len(msg.data) == 0:
                return

            raw = array.array('b', msg.data).tobytes()
            encoded = base64.b64encode(raw).decode('ascii')

            line = (
                f"DATA:MAP:"
                f"{width};{height};{resolution:.6f};{origin_x:.6f};{origin_y:.6f};{encoded}\n"
            )

            self.client_socket.sendall(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending map data: {e}")
            self.disconnect_client()

    def on_pose(self, msg: PoseWithCovarianceStamped):
        """
        Receive robot pose in map frame and forward it to the client.

        Format sent:
        DATA:POSE:<x>;<y>;<yaw>\n
        """
        if not self.connected or self.client_socket is None:
            return

        try:
            now = time.time()
            if now - self.last_pose_sent_time < self.pose_send_period:
                return
            self.last_pose_sent_time = now

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            # yaw from quaternion
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )

            self.latest_pose_x = x
            self.latest_pose_y = y
            self.latest_pose_yaw = yaw

            line = f"DATA:POSE:{x:.6f};{y:.6f};{yaw:.6f}\n"
            self.client_socket.sendall(line.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f"Error sending pose data: {e}")
            self.disconnect_client()

    def teleop_keepalive_callback(self):
        """
        Continuously re-publish the last velocity command while teleoperation is active.

        Prevents robot_control from stopping due to missing cmd_vel updates.
        """
        if not self.teleoperation_active:
            return

        linear, angular = self.last_published_cmd_vel

        with self.cmd_vel_lock:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_publisher.publish(twist)
####################

#####OTHER FUNCTIONS#####
    def publish_log(self, message):
        """
        Publish a log message both to ROS and console for debugging and traceability.
        """
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
        self.get_logger().info(message)

#####PROBE MOVE FUNCTIONS#####
    def _set_gpio_state(self, up: bool, down: bool):
        """
        Set GPIO outputs for probe control.

        Ensures:
        - mutual exclusion (UP and DOWN cannot be active together)
        - safe and consistent hardware control
        """
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
        """
        Handle 'UP' command:
        - move probe upward
        - ignored during arming delay or HOME sequence
        """
        if time.time() - self.start_time < self.PROBE_ARM_DELAY_S:
            self.publish_log("Probe command ignored during arming delay.")
            return

        if self.home_active:
            self.publish_log("UP command received while HOME is active, ignoring.")
            return

        self.publish_log("Handling UP command (GPIO24 HIGH, GPIO25 LOW).")
        self._set_gpio_state(up=True, down=False)

    def handle_down_command(self):
        """
        Handle 'DOWN' command:
        - move probe downward
        - ignored during arming delay or HOME sequence
        """
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
        Handle 'HOME' command:
        - move probe up for fixed duration (8s)
        - then stop automatically
        - blocks manual control during execution
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
    
    def handle_probe_stop_command(self):
        """
        Handle 'STOP_PROBE' command:
        - immediately stop all probe motion
        - cancels HOME sequence if active
        """
        if self.home_active:
            self.publish_log("STOP_PROBE received: cancelling HOME and stopping probe.")
            self.home_active = False
        else:
            self.publish_log("STOP_PROBE received: stopping probe (UP/DOWN LOW).")

        self._set_gpio_state(up=False, down=False)
####################
    
#####DATA CONNECTION FUNCTIONS#####
    def socket_worker(self):
        """
        Main socket thread handling:
        - client connection acceptance
        - receiving data
        - buffering and parsing messages
        - dispatching commands

        Runs independently from ROS thread.
        """
        while rclpy.ok() and not self.shutdown_flag:
            if not self.connected:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
                    self.client_socket.settimeout(1.0)
                    self.publish_log(f"Connected to {addr}")
                    self.connected = True
                    self.last_client_heartbeat_time = time.time()
                    self.disconnect_time = None
                except socket.timeout:
                    pass  
                except socket.error as e:
                    self.get_logger().error(f"Socket error while accepting connection: {e}")

            if self.connected:
                try:
                    ready = select.select([self.client_socket], [], [], 0.1)
                    if ready[0]:
                        start_time = time.time()  # Start latency monitoring
                        data = self.client_socket.recv(1024)
                        if data:
                            self.rx_buffer += data.decode('utf-8')

                            while '\n' in self.rx_buffer:
                                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    self.process_data(line)
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
        """
        Cleanly disconnect the current client.

        Actions:
        - close socket
        - stop robot motion
        - reset connection flags
        - mark disconnection time for watchdog
        """
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
        self.connected = False
        self.data_stream_enabled = False
        self.last_client_heartbeat_time = 0.0

        if self.robot_state == ROBOT_MAPPING:
            self.disconnect_time = time.time()

    def shutdown(self):
        """
        Gracefully shutdown the node.

        - Stops socket thread
        - Closes server and client sockets
        - Resets GPIO
        - Releases hardware resources
        """
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

    def process_data(self, data):
        """
        Process a single line of incoming client data.

        Handles:
        - mapping commands
        - probe commands
        - telemetry control
        - velocity commands
        - heartbeat updates
        """
        self.publish_log(f"Processing data: {data}")
        
        match data:
            case "MAP_START":
                self.start_mapping()
            case "MAP_STOP_DISCARD":
                self.stop_mapping_discard()
            case "MAP_STOP_SAVE_TEMP":
                self.stop_mapping_save_temp()
            case _ if data.startswith("MAP_SAVE_FINAL:"):
                self.save_mapping_final(data)
            case "APP_HEARTBEAT":
                self.last_client_heartbeat_time = time.time()
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
            case "DATA_STREAM_ON":
                self.data_stream_enabled = True
                self.publish_log("Contamination stream enabled.")
            case "DATA_STREAM_OFF":
                self.data_stream_enabled = False
                self.publish_log("Contamination stream disabled.")
            case "STOP_PROBE":
                self.handle_probe_stop_command()
            case _ if self.looks_like_twist_command(data):
                self.process_twist_command(data)
            case _:
                self.publish_log(f"Unknown command: {data}")
####################

#####MOVE/VELOCITY FUNCTIONS#####
    def looks_like_twist_command(self, data):
        """
        Check if incoming data matches a valid velocity command format.

        Expected:
        "<linear>;<angular>"
        """
        parts = data.split(';')
        if len(parts) != 2:
            return False

        try:
            float(parts[0].strip().replace(',', '.'))
            float(parts[1].strip().replace(',', '.'))
            return True
        except ValueError:
            return False
        
    def process_twist_command(self, data):
        """
        Parse and publish velocity command received from client.

        - Converts string to float
        - Publishes Twist message
        - Updates teleoperation state
        """
        try:
            linear, angular = map(str.strip, data.split(';'))
            linear = float(linear.replace(',', '.'))
            angular = float(angular.replace(',', '.'))

            with self.cmd_vel_lock:
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                self.cmd_vel_publisher.publish(twist)
                self.last_cmd_vel_time = self.get_clock().now()
                self.last_published_cmd_vel = (linear, angular)
                self.teleoperation_active = not (abs(linear) < 1e-4 and abs(angular) < 1e-4)
                self.publish_log(f"Published cmd_vel - Linear: {linear}, Angular: {angular}")

        except ValueError:
            self.publish_log(f"Invalid twist command format: {data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in processing twist command: {e}")

    def send_stop_command(self):
        """
        Immediately stop robot motion by publishing zero velocities.

        Also disables teleoperation state.
        """
        with self.cmd_vel_lock:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.last_published_cmd_vel = (0.0, 0.0)
            self.teleoperation_active = False
            self.publish_log("Published stop command")
####################    

#####MISSION MANAGE FUNCTIONS#####
    def send_pause_command(self):
        """
        Send a pause command to robot via ROS topic.
        """
        msg = String()
        msg.data = "Pause"
        self.pause_request_publisher.publish(msg)
        self.publish_log("Published Pause command")

    def send_continue_command(self):
        """
        Send a resume/continue command to robot via ROS topic.
        """
        msg = String()
        msg.data = "Continue"
        self.pause_request_publisher.publish(msg)
        self.publish_log("Published Continue command")

#####MAPPING MANAGE FUNCTIONS#####
    def start_mapping(self):
        """
        Start SLAM mapping process.

        - creates a new mapping job
        - launches mapping stack
        - sets robot state to MAPPING
        """
        if self.robot_state == ROBOT_MAPPING:
            if self.mapping_process is None:
                self.publish_log("Stale MAPPING state detected. Resetting before starting new mapping.")
                self.robot_state = ROBOT_WAITING
                self.current_mapping_job = None
            else:
                self.publish_log("Mapping already running, ignoring MAP_START.")
                return

        if self.robot_state in (ROBOT_WORKING, ROBOT_PAUSED):
            self.publish_log(f"Robot busy in state {self.robot_state}, refusing MAP_START.")
            return

        self.publish_log("Starting mapping process.")

        try:
            self.current_mapping_job = self.create_mapping_job()
            self.save_current_job_metadata()

            self.mapping_process = subprocess.Popen(
                ['ros2', 'launch', 'puppet_irl_bringup', 'manual_mapping_bringup.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.send_stop_command()
            self.robot_state = ROBOT_MAPPING

            self.publish_log(
                f"Mapping started. Job ID: {self.current_mapping_job['job_id']}"
            )

        except Exception as e:
            self.robot_state = ROBOT_ERROR
            self.publish_log(f"Error starting mapping: {str(e)}")

    def stop_mapping_discard(self):
        """
        Stop mapping and discard results.

        - stops SLAM stack
        - marks job as DISCARDED
        """
        self.publish_log("Stopping mapping without saving preview.")
        self.send_stop_command()
        self._stop_mapping_stack()

        if self.current_mapping_job is not None:
            self.current_mapping_job["status"] = "DISCARDED"
            self.save_current_job_metadata()
            self.current_mapping_job = None

        self.robot_state = ROBOT_WAITING
        self.disconnect_time = None

    def stop_mapping_save_temp(self):
        """
        Stop mapping and save a temporary preview map.

        - calls map_saver_cli
        - updates job status
        """
        self.publish_log("Stopping mapping and saving temporary preview.")
        self.send_stop_command()

        if self.current_mapping_job is None:
            self.publish_log("No active mapping job, cannot save preview.")
            self._stop_mapping_stack()
            self.robot_state = ROBOT_WAITING
            return

        temp_map_base = self.current_mapping_job["temp_map_base"]

        try:
            time.sleep(2)
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', temp_map_base],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False
            )
            self.current_mapping_job["status"] = "PREVIEW_READY"
            self.save_current_job_metadata()
            self.publish_log(f"Temporary map saved to {temp_map_base}")
        except Exception as e:
            self.current_mapping_job["status"] = "SAVE_TEMP_FAILED"
            self.save_current_job_metadata()
            self.publish_log(f"Error saving temporary map: {e}")

        self._stop_mapping_stack()
        self.robot_state = ROBOT_WAITING

    def _stop_mapping_stack(self):
        """
        Stop SLAM mapping stack processes.

        Uses system-level process kill (pkill).
        """
        try:
            self.publish_log("Stopping mapping stack using pkill -f nav2 ...")

            result = subprocess.run(
                ['pkill', '-f', 'slam'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False
            )

            self.publish_log(f"pkill return code: {result.returncode}")
            self.mapping_process = None
            self.publish_log("Mapping stack stopped.")

        except Exception as e:
            self.publish_log(f"Error stopping mapping stack: {e}")

    def save_current_job_metadata(self):
        """
        Save current mapping job metadata to disk (job.json).
        """
        if self.current_mapping_job is None:
            return

        meta_path = os.path.join(self.current_mapping_job["job_dir"], "job.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(self.current_mapping_job, f, indent=2)

    def save_mapping_final(self, data):
        """
        Handle final map save request from client.

        Expected format:
        MAP_SAVE_FINAL:<map_key>;<display_name>

        Responsible for:
        - saving latest map to disk
        - stopping mapping cleanly
        - finalizing job
        """
        try:
            payload = data.split(":", 1)[1]
            map_key, display_name = [x.strip() for x in payload.split(";", 1)]
        except Exception:
            self.publish_log(f"Invalid MAP_SAVE_FINAL format: {data}")
            return

    def create_mapping_job(self):
        """
        Create a new mapping job structure and directory.

        Returns metadata dictionary describing the job.
        """
        job_id = datetime.now().strftime("job_%Y%m%d_%H%M%S")
        job_dir = os.path.join(self.jobs_root, job_id)
        os.makedirs(job_dir, exist_ok=True)

        return {
            "job_id": job_id,
            "job_dir": job_dir,
            "temp_map_base": os.path.join(job_dir, "temp_map"),
            "status": "RUNNING",
            "created_at": datetime.now().isoformat()
        }
####################
####################
#########################

########MAIN########
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
####################
