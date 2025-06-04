import smbus2
import struct
import time

# I2C bus number (usually 1 for Raspberry Pi)
I2C_BUS = 1

# I2C address of the Arduino
I2C_SLAVE_ADDRESS = 0x0B  # The I2C address of the slave device (11 in decimal is 0x0B in hex)

def send_twist(linear_x, angular_z):
    # Create a byte array from the floats
    data = struct.pack('ff', linear_x, angular_z)
    data_length = len(data)
    
    print(f'Data length: {data_length} bytes')  # Should be 8 bytes
    print(f'Data: {list(data)}')
    
    if data_length != 8:
        print('Error: Data length is not 8 bytes')
        return
    
    # Create the I2C bus
    bus = smbus2.SMBus(I2C_BUS)
    
    try:
        # Send the data via I2C
        bus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0, list(data))
        print(f'Sent linear_x: {linear_x}, angular_z: {angular_z}')
    except Exception as e:
        print(f'Error sending data: {e}')
    finally:
        bus.close()

def main():
    print("Sending Twist messages via I2C...")
    
    # Send Twist with linear_x = 0.1 and angular_z = 0.0
    send_twist(0.1, 0.0)
    
    # Wait to ensure the message is received and processed
    time.sleep(1)

if __name__ == '__main__':
    main()
