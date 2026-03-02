import RPi.GPIO as GPIO
import time

I2C_RESET_PIN = 16  # GPIO16

# Setup
GPIO.setmode(GPIO.BCM)  # Use BCM numbering
GPIO.setup(I2C_RESET_PIN, GPIO.OUT)

# Reset sequence
GPIO.output(I2C_RESET_PIN, GPIO.LOW)  # Set to LOW
time.sleep(0.1)  # Keep it LOW for 100ms
GPIO.output(I2C_RESET_PIN, GPIO.HIGH)  # Set to HIGH (release reset)

# Clean up
GPIO.cleanup()
