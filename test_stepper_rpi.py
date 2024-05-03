import RPi.GPIO as GPIO
import time

# Set up GPIO pins
def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme
    GPIO.setup(2, GPIO.OUT)
    GPIO.setup(3, GPIO.OUT)
    GPIO.setup(4, GPIO.OUT)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)

    # Set GPIO 2, 3, 4 to HIGH
    GPIO.output(2, GPIO.HIGH)
    GPIO.output(3, GPIO.HIGH)
    GPIO.output(4, GPIO.HIGH)

# Function to control stepper motor
def control_stepper(steps, direction):
    """ Control a stepper motor.
    Args:
        steps (int): Number of steps to rotate the stepper motor.
        direction (bool): True for one direction, False for the opposite.
    """
    GPIO.output(6, GPIO.HIGH if direction else GPIO.LOW)  # Set direction
    for _ in range(steps):
        GPIO.output(5, GPIO.HIGH)
        time.sleep(0.01)  # Step pulse duration
        GPIO.output(5, GPIO.LOW)
        time.sleep(0.01)  # Time between steps

# Example usage
try:
    setup_gpio()
    # Example: Rotate stepper 100 steps in one direction
    control_stepper(100, True)
finally:
    GPIO.cleanup()  # Clean up GPIO on exit

