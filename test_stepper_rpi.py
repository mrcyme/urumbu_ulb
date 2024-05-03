from gpiozero import OutputDevice
from time import sleep

# Setup GPIO devices
gpio2 = OutputDevice(2, initial_value=True)
gpio3 = OutputDevice(3, initial_value=True)
gpio4 = OutputDevice(4, initial_value=True)
step_pin = OutputDevice(5)
dir_pin = OutputDevice(6)

def control_stepper(steps, direction):
    """ Control a stepper motor using GPIO Zero.
    Args:
        steps (int): Number of steps to rotate the stepper motor.
        direction (bool): True for one direction, False for the opposite.
    """
    dir_pin.value = 1 if direction else 0  # Set direction
    for _ in range(steps):
        step_pin.on()
        sleep(0.01)  # Step pulse duration
        step_pin.off()
        sleep(0.01)  # Time between steps

# Example usage
control_stepper(100, True)