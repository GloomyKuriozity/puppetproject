#!/usr/bin/env python3

import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from . import read_sfp100_rate


class Sfp100Node(Node):
    def __init__(self):
        super().__init__('sfp100_node')

        # Parameters
        self.declare_parameter('port', '/dev/sfp100')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('period', 0.5)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        period = self.get_parameter('period').get_parameter_value().double_value

        # Serial state
        self.ser = None
        self.serial_ok = False
        self.reconnect_interval = 5.0
        self.last_reconnect_attempt = 0.0

        self.open_serial_once()

        # Publish only raw cps values:
        # [alpha_cps, beta_gamma_cps]
        self.pub = self.create_publisher(Float32MultiArray, 'sfp100/contamination', 10)
        self.timer = self.create_timer(period, self.read_and_publish)

    def open_serial_once(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.6,
                write_timeout=0.6,
            )
            self.serial_ok = True
            self.get_logger().info(f"Opened SFP-100 on {self.port} @ {self.baud} Bd")
        except serial.SerialException as e:
            self.ser = None
            self.serial_ok = False
            self.last_reconnect_attempt = time.time()
            self.get_logger().warn(f"Could not open serial port {self.port}: {e}")

    def read_and_publish(self):
        if not self.serial_ok:
            now = time.time()
            if now - self.last_reconnect_attempt >= self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.get_logger().warn("Serial not OK, trying to reopen...")
                self.open_serial_once()
            return

        try:
            beta_gamma_cps, alpha_cps = read_sfp100_rate.read_two_rates_once(self.ser)

        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial error, will try to reconnect: {e}")
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass

            self.ser = None
            self.serial_ok = False
            self.last_reconnect_attempt = time.time()
            return

        except TimeoutError:
            return

        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")
            return

        alpha_cps = float(alpha_cps)
        beta_gamma_cps = float(beta_gamma_cps)

        msg = Float32MultiArray()
        msg.data = [
            alpha_cps,
            beta_gamma_cps,
        ]
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sfp100Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()