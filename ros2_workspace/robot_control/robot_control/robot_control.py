'''
Author: Mélanie Geulin

Last Update: 08/07/2025

Script: robot_control

Summary: Process speed command from PC
         Process sonar and odometry data from robot
'''

import rclpy
import smbus2
import struct
import math
import time
import threading
import numpy as np
from std_msgs.msg import String
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.executors import MultiThreadedExecutor

CMD_VEL_GRACE_PERIOD = 0.8
MIN_THETA_CHANGE = 0.0001  # radians, ~0.057°

BASE_IS_BACK_OFFSET_RAD = 0.0
THETA_OFFSET_RAD = math.pi / 2

class CmdVelToCAN(Node):
#####INITIALIZATION#####
    def __init__(self):
        super().__init__('robot_control')

        # I2C setup
        self.get_logger().info('Init I2C...')
        self.bus = smbus2.SMBus(1)      # I2C 1 bus number velocities + charger
        self.bus_2 = smbus2.SMBus(3)    # I2C 2 bus number odometry
        self.arduino_address = 0x11     # I2C 1 address of the Arduino (11 in decimal)
        self.arduino_address_2 = 0x22   # I2C 2 address of the Arduino (22 in decimal)
        self.get_logger().info('I2C init complete!')

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.33, self.timer_callback)
        #self.log_publisher = self.create_publisher(String, 'logs', 10)

        self.br = TransformBroadcaster(self)    # Transform buffer
        self.cmd_vel_queue = deque(maxlen=10)   # Buffer for cmd_vel messages
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_event = threading.Event()
        self.has_sent_stop = False

        self.prev_x_pose = 0
        self.prev_y_pose = 0
        self.prev_theta_pose = 0
        self.prev_linear_vel = 0
        self.prev_angular_vel = 0
        self.last_odom_msg_time = 0

        self.real_linear_vel = 0
        self.real_angular_vel = 0
        self.current_angular_vel = 0
        self.current_linear_vel = 0
        self.too_long_reactivity = self.get_clock().now()

        # Kalman filter state for fallback odometry
        '''self.kf_pose = np.zeros((3, 1))  # [x, y, theta]
        self.kf_P = np.eye(3) * 0.1      # Initial covariance
        self.kf_Q = np.eye(3) * 0.01     # Process noise covariance'''

        # Create a dedicated thread for cmd_vel processing
        self.cmd_vel_thread = threading.Thread(target=self.cmd_vel_thread_worker, daemon=True)
        self.cmd_vel_thread.start()
########################

#####CALLBACKS######
    def cmd_vel_callback(self, msg):
        '''
        Callback command vel received by PC/app_command
        '''
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.cmd_vel_queue.append((linear_x, angular_z))
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_event.set()

    def timer_callback(self):
        '''
        Manage odometry received from arduino
        '''
        #Security stop handler
        time_since_last_cmd_vel = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9

        if time_since_last_cmd_vel > 1.0:
            if not self.has_sent_stop:
                self.get_logger().warning(f"No cmd_vel received for {time_since_last_cmd_vel:.2f} seconds, sending one-time stop.")
                self.send_stop_command()
                self.has_sent_stop = True
        else:
            self.has_sent_stop = False  # Reset stop flag on recent cmd_vel

        if (self.real_angular_vel > 0.0 or self.real_linear_vel > 0.0) and \
        (self.current_linear_vel == 0.0 and self.current_angular_vel == 0.0):

            time_elapsed = (self.get_clock().now() - self.too_long_reactivity).nanoseconds / 1e9
            if time_elapsed > 1.0:
                self.get_logger().warn("Detected I2C freeze: velocities sent but robot shows no odometry. Triggering I2C reset.")
                #self.reset_i2c()
                self.too_long_reactivity = self.get_clock().now()
        else:
            self.too_long_reactivity = self.get_clock().now()

        # Proceed with reading data from Arduino
        try:
            now = self.get_clock().now()
            data = self.read_i2c2_with_retries(21)
            if not self.check_data_coherence(data, now):
                # Allow fallback TF if it's been at least 1 second since boot
                time_since_start = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9

                if self.last_odom_msg_time.nanoseconds == 0 and time_since_start < 1.0:
                    #self.get_logger().warn("Skipping TF publish at startup – no valid odometry yet.")
                    return  # Still too early, don't publish yet

                pose_array = np.array([[self.prev_x_pose], [self.prev_y_pose], [self.prev_theta_pose]])
                self.publish_odom_messages_tf(pose_array, self.prev_linear_vel, self.prev_angular_vel, now, source="ODOM")

        except Exception as e:
            self.get_logger().error(f"Error reading I2C data: {e}")
####################


#####THREAD WORKER#####
    def cmd_vel_thread_worker(self):
        '''
        Separate thread to handle the cmd_vel commands.
        It continuously checks the cmd_vel_queue and processes the commands.
        '''
        while rclpy.ok():
            # Wait for an event or timeout
            if self.cmd_vel_event.wait(timeout=CMD_VEL_GRACE_PERIOD):  # Wait for up to 1 second
                while self.cmd_vel_queue:
                    linear_x, angular_z = self.cmd_vel_queue.popleft()
                    message = struct.pack('ff', linear_x, angular_z)
                    self.send_via_i2c_1(message)
                    time.sleep(0.05)  # 50ms between I²C messages

                # Reset the event once the queue is empty
                self.has_sent_stop = False
                self.cmd_vel_event.clear()

#######################

#####OTHER FUNCTIONS#####
    '''def reset_i2c(self):
        try:
            self.get_logger().warn("Calling reset_i2c.py...")
            import subprocess
            subprocess.run(['sudo', 'python3', '/home/mgeulin/ros2_ws/reset_i2c.py'],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
            self.get_logger().info("I2C reset script executed successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset I2C: {e}")'''

    def send_via_i2c_1(self, message, max_retries=3):
        for attempt in range(max_retries):
            try:
                self.bus.write_i2c_block_data(self.arduino_address, 0, list(message))

            except Exception as e:
                self.get_logger().error(f"send_via_i2c_1 failed: {e}")
                if attempt + 1 == max_retries:
                    self.get_logger().error('Max retries reached, could not send I2C message')

    def read_i2c2_with_retries(self, length, max_retries=3):
        backoff = 1
        for attempt in range(max_retries):
            try:
                data = self.bus_2.read_i2c_block_data(self.arduino_address_2, 0, length)
                return data
            except Exception as e:
                self.get_logger().error(f"Attempt {attempt + 1} to read I2C2 failed: {e}")
                if attempt + 1 == max_retries:
                    raise e  # Re-raise after max retries
                else:
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 32)  # Cap the backoff

    def send_stop_command(self):
        '''
        Send a stop command to the robot by setting velocities to zero.
        '''
        message = struct.pack('ff', 0.0, 0.0)  # Zero velocities
        self.send_via_i2c_1(message)
        self.real_angular_vel = 0.0
        self.real_linear_vel = 0.0

    def is_valid_number(self, x):
        return not (math.isnan(x) or math.isinf(x))

    def check_data_coherence(self, data, now):
        # Check if data has the correct length
        if len(data) != 21:
            self.get_logger().warn(f"Received data of invalid length: {len(data)} bytes.")
            return False

        try:
            x, y, theta, linear_velocity, angular_velocity = struct.unpack('<fffff', bytearray(data[:20]))
            self.current_linear_vel = linear_velocity
            self.current_angular_vel = angular_velocity

            # Check for NaN, Inf, or all-zero values
            if not all(self.is_valid_number(val) for val in [x, y, theta, linear_velocity, angular_velocity]):
                self.get_logger().warn("Odometry contains NaN or Inf values.")
                return False

            # Check for repeated pose
            # Compare rounded values to 0.0001 precision (0.1 mm)
            if (round(x, 4) == round(self.prev_x_pose, 4) and
                round(y, 4) == round(self.prev_y_pose, 4) and
                abs(theta - self.prev_theta_pose) < MIN_THETA_CHANGE):
                if self.real_angular_vel != 0 or self.real_linear_vel != 0:
                    self.get_logger().warn("Odometry data is unchanged from previous reading but robot is moving.")
                    return False
                else:
                    self.get_logger().warn("Odometry data is unchanged from previous reading but robot is NOT moving.")

            # Update previous pose for future comparison
            self.prev_x_pose = round(x, 4)
            self.prev_y_pose = round(y, 4)
            self.prev_theta_pose = theta
            self.prev_linear_vel = linear_velocity
            self.prev_angular_vel = angular_velocity
            self.last_odom_msg_time = now

            pose_array = np.array([[x], [y], [theta]])
            self.publish_odom_messages_tf(pose_array, linear_velocity, angular_velocity, now, source="ODOM")

            return True

        except struct.error as e:
            self.get_logger().error(f"Error unpacking odometry data: {e}")
            return False

    def publish_odom_messages_tf(self, pose, v, w, now, source):
        x_a = pose[0, 0]
        y_a = pose[1, 0]
        theta_a = pose[2, 0]

        x =  y_a
        y = -x_a
        theta = theta_a - math.pi/2

        # Wrap theta to [-pi, pi]
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        # Quaternion from pure yaw (NO offsets)
        s = math.sin(theta / 2.0)
        c = math.cos(theta / 2.0)

        # ---- Odometry message ----
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = s
        odom_msg.pose.pose.orientation.w = c

        odom_msg.twist.twist.linear.x = v        # body-frame forward (+X)
        odom_msg.twist.twist.linear.y = 0.0      # diff drive: no lateral velocity in body frame
        odom_msg.twist.twist.angular.z = w

        self.odom_publisher.publish(odom_msg)

        # ---- TF message ----
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = s
        t.transform.rotation.w = c

        self.br.sendTransform(t)

        self.get_logger().info(
            f"[TF] odom->base_link at x={x:.2f}, y={y:.2f}, theta(raw)={pose[2,0]:.3f}, yaw(used)={theta:.3f}"
        )

#########################

#####MAIN#####
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToCAN()

    # Use MultiThreadedExecutor to allow separate processing of cmd_vel and odom callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Initialized robot_control with multi-threading!")
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
##############