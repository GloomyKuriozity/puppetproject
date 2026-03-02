#!/usr/bin/env python3

import serial
import struct  # not strictly needed here, but kept in case you extend
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Import your existing script as a module
from . import read_sfp100_rate  # make sure this is on PYTHONPATH / installed as a module


# Typical response factors (approx) from the manual
#   Alpha (Am-241): ~0.21 cps/Bq  → RF ≈ 1 / 0.21 Bq/cps
#   Beta (Cl-36):   ~0.32 cps/Bq  → RF ≈ 1 / 0.32 Bq/cps
RF_ALPHA_BQ_PER_CPS = 1.0 / 0.21   # ≈ 4.7619 Bq/cps
RF_BETA_BQ_PER_CPS  = 1.0 / 0.32   # ≈ 3.125  Bq/cps

DETECTOR_AREA_CM2   = 100.0        # SFP-100 active area


class Sfp100Node(Node):
    def __init__(self):
        super().__init__('sfp100_node')

        # --- Parameters for serial config ---
        self.declare_parameter('port', '/dev/sfp100')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('period', 0.5)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        period = self.get_parameter('period').get_parameter_value().double_value

        # --- serial state ---
        self.ser = None
        self.serial_ok = False
        self.reconnect_interval = 5.0   # seconds between reconnect attempts
        self.last_reconnect_attempt = 0.0

        self.open_serial_once()         # try initial open

        # publisher + timer as before
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
            self.get_logger().warn(f"Could not open serial port {self.port}: {e}")
            self.last_reconnect_attempt = time.time()

    def extract_frames(self, buffer: bytearray):
        """
        Same idea as in your old script: find '#' ... '\r' frames.
        Returns a list of complete frames (bytes).
        """
        frames = []
        while True:
            i_hash = buffer.find(b"#")
            if i_hash == -1:
                buffer.clear()
                break

            if i_hash > 0:
                del buffer[:i_hash]  # drop anything before '#'

            i_cr = buffer.find(b"\r", 1)
            if i_cr == -1:
                # Incomplete frame, wait for more bytes
                break

            frames.append(bytes(buffer[:i_cr + 1]))  # include '\r'
            del buffer[:i_cr + 1]
        return frames

    def read_and_publish(self):
        import time
        if not self.serial_ok:
            # try to reopen every self.reconnect_interval seconds
            now = time.time()
            if now - self.last_reconnect_attempt >= self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.get_logger().warn("Serial not OK, trying to reopen...")
                self.open_serial_once()
            return  # nothing to do this tick

        # Serial is (supposedly) OK, try a measurement
        try:
            beta_gamma_cps, alpha_cps = read_sfp100_rate.read_two_rates_once(self.ser)
        except (serial.SerialException, OSError) as e:
            # USB cable yanked, power drop, etc.
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
            # harmless, just skip this cycle
            return
        except Exception as e:
            # Any protocol/parse error, log and skip
            self.get_logger().warn(f"Read error: {e}")
            return

        # --- if we reach here, we have fresh cps values ---
        alpha_cps = float(alpha_cps)
        beta_gamma_cps = float(beta_gamma_cps)

        alpha_bq_cm2 = alpha_cps * RF_ALPHA_BQ_PER_CPS / DETECTOR_AREA_CM2
        beta_gamma_bq_cm2 = beta_gamma_cps * RF_BETA_BQ_PER_CPS / DETECTOR_AREA_CM2

        msg = Float32MultiArray()
        msg.data = [
            alpha_cps,
            alpha_bq_cm2,
            beta_gamma_cps,
            beta_gamma_bq_cm2,
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
