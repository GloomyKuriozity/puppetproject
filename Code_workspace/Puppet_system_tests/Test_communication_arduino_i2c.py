import smbus
import struct
import time

I2C_SLAVE_ADDRESS = 11
bus = smbus.SMBus(1)

def read_odometry():
    while True:
        try:
            data = bus.read_i2c_block_data(I2C_SLAVE_ADDRESS, 0, 20)  # 5 floats * 4 bytes each = 20 bytes
            if data:
                x, y, theta, linear_velocity, angular_velocity = struct.unpack('<fffff', bytearray(data))
                return x, y, theta, linear_velocity, angular_velocity
        except OSError:
            time.sleep(0.01)  # Small delay before retrying

while True:
    x, y, theta, linear_velocity, angular_velocity = read_odometry()
    print(f"x: {x}, y: {y}, theta: {theta}, linear_velocity: {linear_velocity}, angular_velocity: {angular_velocity}")
    time.sleep(0.1)  # Adjust as necessary
