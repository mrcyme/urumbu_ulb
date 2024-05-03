import gpiod
import time

# Define the chip and lines
CHIP = 'gpiochip0'
LINE_VALUES_HIGH = [2, 3, 4]  # GPIO pins to be set high
STEP_PIN = 17  # Step signal for the stepper motor
DIR_PIN = 27  # Direction signal for the stepper motor

# Setup GPIO
def setup_gpio():
    chip = gpiod.Chip(CHIP)

    # Set lines 2, 3, 4 to high
    lines = chip.get_lines(LINE_VALUES_HIGH)
    lines.request(consumer='gpio_set_high', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1]*len(LINE_VALUES_HIGH))

    return chip

# Control stepper motor function
def control_stepper(chip, steps, direction):
    """ Control a stepper motor using libgpiod.
    Args:
        chip (gpiod.Chip): gpiod chip instance.
        steps (int): Number of steps to rotate the stepper motor.
        direction (bool): True for clockwise, False for counterclockwise.
    """
    step_line = chip.get_line(STEP_PIN)
    dir_line = chip.get_line(DIR_PIN)

    step_line.request(consumer='step_line', type=gpiod.LINE_REQ_DIR_OUT)
    dir_line.request(consumer='dir_line', type=gpiod.LINE_REQ_DIR_OUT)

    # Set direction
    dir_line.set_value(1 if direction else 0)

    # Send pulses to the step line
    for _ in range(steps):
        step_line.set_value(1)
        time.sleep(0.01)  # Step pulse duration
        step_line.set_value(0)
        time.sleep(0.01)  # Time between steps

# Main program logic
if __name__ == '__main__':
    chip = setup_gpio()
    try:
        control_stepper(chip, 100, True)  # 100 steps, clockwise
    finally:
        chip.close()  # It's important to clean up and close the chip
