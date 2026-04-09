'''
Author: Melanie Geulin

Mail: melanie.geulin@orano.group

Last Update: 09/04/2026

Script: robot_control

Summary: Process speed command from PC
         Process sonar and odometry data from robot
'''

import rclpy
import smbus2
import struct
import math
import time
import numpy as np
from std_msgs.msg import String
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MIN_THETA_CHANGE = 0.0001           # radians, ~0.057°

class CmdVelToCAN(Node):
#####INITIALIZATION#####
    def __init__(self):
        super().__init__('robot_control')

        ####INIT CONNEXION Arduino/RP5####
        self.bus = smbus2.SMBus(1)      # I2C 1 bus number velocities + charger
        self.bus_2 = smbus2.SMBus(3)    # I2C 2 bus number odometry
        self.arduino_address = 0x11     # I2C 1 address of the Arduino (11 in decimal)
        self.arduino_address_2 = 0x22   # I2C 2 address of the Arduino (22 in decimal)

        ####PUBLISHERS####
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.robot_info_publisher = self.create_publisher(String, 'robot_info', 10)

        ####SUBSCRIBER####
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, cmd_vel_qos)

        ####TIMER####
        self.timer = self.create_timer(0.33, self.timer_callback)
        
        ####ODOM VARIABLES####
        self.br = TransformBroadcaster(self) 
        self.prev_x_pose = 0
        self.prev_y_pose = 0
        self.prev_theta_pose = 0
        self.prev_linear_vel = 0
        self.prev_angular_vel = 0
        self.last_odom_msg_time = None

        ####VELOCITY VARIABLES####
        self.has_sent_stop = False
        self.real_linear_vel = 0
        self.real_angular_vel = 0
        self.current_angular_vel = 0
        self.current_linear_vel = 0

        ####BATTERY VARIABLES####
        self.battery_voltage = float('nan')
        self.battery_percentage = None
        self.battery_low_warning = False
        self.battery_critical_warning = False

        ####SYNC CHECKER VARIABLES####
        self.start_time = self.get_clock().now()
        self.too_long_reactivity = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
########################

#####CALLBACKS######
    def cmd_vel_callback(self, msg):
        """
        Receive a velocity command from ROS, forward it immediately to the Arduino over I2C,
        and update the latest commanded linear and angular speeds.

        Also marks whether the latest received command is a full stop.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.last_cmd_vel_time = self.get_clock().now()

        message = struct.pack('ff', linear_x, angular_z)
        self.send_via_i2c_1(message)

        self.real_linear_vel = linear_x
        self.real_angular_vel = angular_z
        self.has_sent_stop = (abs(linear_x) < 1e-4 and abs(angular_z) < 1e-4)

    def timer_callback(self):
        """
        Periodic supervisor callback.

        Responsibilities:
        - detect cmd_vel timeout and send a one-time stop command if needed
        - monitor command/odometry consistency
        - read odometry and battery data from Arduino over I2C
        - validate received data
        - publish odometry and TF, or reuse previous valid pose if current data is incoherent
        """
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
                self.too_long_reactivity = self.get_clock().now()
        else:
            self.too_long_reactivity = self.get_clock().now()

        # Proceed with reading data from Arduino
        try:
            now = self.get_clock().now()
            data = self.read_i2c2_with_retries(24)
            if not self.check_data_coherence(data, now):
                # Allow fallback TF if it's been at least 1 second since boot
                time_since_start = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

                if self.last_odom_msg_time is None and time_since_start < 1.0:
                    return

                pose_array = np.array([[self.prev_x_pose], [self.prev_y_pose], [self.prev_theta_pose]])
                self.publish_odom_messages_tf(pose_array, self.prev_linear_vel, self.prev_angular_vel, now, source="ODOM")

        except Exception as e:
            self.get_logger().error(f"Error reading I2C data: {e}")
####################

#####OTHER FUNCTIONS#####

#####ARDUINO COMMUNICATION#####
    def send_via_i2c_1(self, message, max_retries=3):
        """
        Send a raw byte message to the velocity Arduino on I2C bus 1.

        Retries a few times before reporting a permanent communication failure.
        """
        for attempt in range(max_retries):
            try:
                self.bus.write_i2c_block_data(self.arduino_address, 0, list(message))

            except Exception as e:
                self.get_logger().error(f"send_via_i2c_1 failed: {e}")
                if attempt + 1 == max_retries:
                    self.get_logger().error('Max retries reached, could not send I2C message')

    def read_i2c2_with_retries(self, length, max_retries=3):
        """
        Read a fixed-length byte payload from the odometry Arduino on I2C bus 3.

        Retries with exponential backoff before raising the error.
        """
        backoff = 1
        for attempt in range(max_retries):
            try:
                read = smbus2.i2c_msg.read(self.arduino_address_2, length)
                self.bus_2.i2c_rdwr(read)
                return list(read)
            except Exception as e:
                self.get_logger().error(f"Attempt {attempt + 1} to read I2C2 failed: {e}")
                if attempt + 1 == max_retries:
                    raise
                time.sleep(backoff)
                backoff = min(backoff * 2, 32)
#########################

#####VELOCITY FUNCTIONS#####          
    def send_stop_command(self):
        """
        Send an immediate zero-velocity command to the robot over I2C.

        Also resets the internally tracked commanded linear and angular velocities.
        """
        message = struct.pack('ff', 0.0, 0.0)  # Zero velocities
        self.send_via_i2c_1(message)
        self.real_angular_vel = 0.0
        self.real_linear_vel = 0.0
#########################

#####ODOM FUNCTIONS#####
    def publish_odom_messages_tf(self, pose, v, w, now, source):
        """
        Publish odometry and odom->base_link TF from the given pose and velocity values.

        Applies the coordinate transform used by this robot setup before publishing.
        """
        x_a = float(pose[0, 0])
        y_a = float(pose[1, 0])
        theta_a = float(pose[2, 0])

        v = float(v)
        w = float(w)

        x = float(y_a)
        y = float(-x_a)
        theta = float(theta_a - math.pi / 2.0)

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
        
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = float(s)
        odom_msg.pose.pose.orientation.w = float(c)

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = float(w)

        self.odom_publisher.publish(odom_msg)

        # ---- TF message ----
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(s)
        t.transform.rotation.w = float(c)

        self.br.sendTransform(t)

        self.get_logger().info(
            f"[TF] odom->base_link at x={x:.2f}, y={y:.2f}, theta(raw)={pose[2,0]:.3f}, yaw(used)={theta:.3f}"
        )
#########################

#####BATTERY FUNCTIONS#####
    def update_battery_state(self):
        """
        Update battery percentage and warning flags from the current battery voltage.

        Sets low and critical battery flags according to the configured thresholds.
        """
        if not self.is_valid_number(self.battery_voltage):
            self.battery_percentage = None
            self.battery_low_warning = False
            self.battery_critical_warning = False
            return

        self.battery_percentage = self.voltage_to_percentage(self.battery_voltage)

        if self.battery_percentage is None:
            self.battery_low_warning = False
            self.battery_critical_warning = False
            return

        self.battery_low_warning = self.battery_percentage < 15.0
        self.battery_critical_warning = self.battery_percentage < 5.0

    def publish_robot_info(self):
        """
        Publish current battery telemetry as a formatted string on the 'robot_info' topic.

        Includes:
        - battery voltage
        - battery percentage
        - low battery warning
        - critical battery warning
        """
        msg = String()

        if self.battery_percentage is None:
            msg.data = f"battery_voltage={self.battery_voltage:.2f};battery_percentage=unknown;low_warning=0;critical_warning=0"
        else:
            msg.data = (
                f"battery_voltage={self.battery_voltage:.2f};"
                f"battery_percentage={self.battery_percentage:.1f};"
                f"low_warning={int(self.battery_low_warning)};"
                f"critical_warning={int(self.battery_critical_warning)}"
            )

        self.robot_info_publisher.publish(msg)

    def voltage_to_percentage(self, voltage):
        """
        Convert battery voltage into an estimated battery percentage using the configured
        piecewise interpolation curve.
        """
        if not self.is_valid_number(voltage):
            return None

        curve = [
            (29.2, 100),
            (28.0, 90),
            (27.0, 75),
            (26.0, 65),
            (25.0, 55),
            (24.0, 50),
            (23.0, 30),
            (22.0, 15),
            (21.0, 0),
        ]

        # Clamp
        if voltage >= curve[0][0]:
            return 100.0
        if voltage <= curve[-1][0]:
            return 0.0

        # Interpolate
        for i in range(len(curve) - 1):
            v_high, p_high = curve[i]
            v_low, p_low = curve[i + 1]

            if v_low <= voltage <= v_high:
                ratio = (voltage - v_low) / (v_high - v_low)
                return round(p_low + ratio * (p_high - p_low), 1)

        return None
#########################

#####COHERENCE CHECKER FUNCTIONS#####
    def is_valid_number(self, x):
        """
        Return True if the given numeric value is finite and usable.

        Rejects NaN and infinite values.
        """
        return not (math.isnan(x) or math.isinf(x))

    def check_data_coherence(self, data, now):
        """
        Validate and unpack the odometry/battery packet received from Arduino.

        Checks:
        - expected packet size
        - finite odometry values
        - repeated pose inconsistencies while motion is commanded

        If valid:
        - updates cached pose/velocity values
        - updates battery state
        - publishes odometry and TF

        Returns True if the data is coherent, otherwise False.
        """
        if len(data) != 24:
            self.get_logger().warn(f"Received data of invalid length: {len(data)} bytes.")
            return False

        try:
            x, y, theta, linear_velocity, angular_velocity, battery_voltage = struct.unpack('<ffffff', bytearray(data[:24]))

            if not all(self.is_valid_number(val) for val in [x, y, theta, linear_velocity, angular_velocity]):
                self.get_logger().warn("Odometry contains NaN or Inf values.")
                self.get_logger().warn(f"Raw unpacked values: x={x}, y={y}, theta={theta}, v={linear_velocity}, w={angular_velocity}, batt={battery_voltage}")
                return False

            self.current_linear_vel = linear_velocity
            self.current_angular_vel = angular_velocity

            # Battery handling
            if self.is_valid_number(battery_voltage) and battery_voltage >= 0.0:
                self.battery_voltage = battery_voltage
            else:
                self.battery_voltage = float('nan')

            self.update_battery_state()
            self.publish_robot_info()

            if (round(x, 4) == round(self.prev_x_pose, 4) and
                round(y, 4) == round(self.prev_y_pose, 4) and
                abs(theta - self.prev_theta_pose) < MIN_THETA_CHANGE):
                if self.real_angular_vel != 0 or self.real_linear_vel != 0:
                    self.get_logger().warn("Odometry data is unchanged from previous reading but robot is moving.")
                    return False
                else:
                    self.get_logger().warn("Odometry data is unchanged from previous reading but robot is NOT moving.")

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
#########################
#########################

#####MAIN#####
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToCAN()

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
