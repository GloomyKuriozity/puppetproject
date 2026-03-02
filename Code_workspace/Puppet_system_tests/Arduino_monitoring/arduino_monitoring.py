import serial
import time

# Open port and force reset
ser = serial.Serial('/dev/ttyUSB0', 9600)
ser.setDTR(False)       # Disable DTR
time.sleep(0.1)         # Wait briefly
ser.setDTR(True)        # Enable DTR - this causes the Arduino to reset
time.sleep(2)           # Give Arduino time to reboot

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(line)
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
