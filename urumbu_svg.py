# Modified version of Quentin Bolsee 'urumbu_gcode.py'.  
# Modified by Robert Hart 1/13/22 - 1/14/22.  
# Used 1/14/22 to run Urumbu machine with x,y, and a cam-driven z-axis.  
# Supports g-code in a limited way.
#
#
#
import serial
import time
import multiprocessing
import logging
import argparse
import numpy as np
import math
import os
import matplotlib.pyplot as plt
from pyaxidraw import axidraw

BAUDRATE_DEFAULT = 921600

class Module:
    def __init__(self, port, preview, baudrate=BAUDRATE_DEFAULT, com="serial"):
        self.com = com
        self.port = None
        self.ip = None
        self.baudrate = baudrate
        self.preview = preview
        if not self.preview:
            try:
                self.port = serial.Serial(port, baudrate)
            except serial.SerialException:
                logging.error(f"Cannot connect to {port}")

    @property
    def connected(self):
        return self.port is not None

    def write(self, txt):
        if self.com == "serial":
            self.port.write(txt)
        if self.com == "rpi_gpio":
            pass

    def close(self):
        self.port.close()

    def pressed(self, nc=True):
        self.write(b"?")
        r = self.port.read(1)
        if nc:
            return r == b"1"
        else:
            return r == b"0"


def gpio_control_stepper(steps, direction):
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

class Stepper(Module):
    def __init__(self, steps_per_unit, port, step_pin=None, dir_pin = None, baudrate=BAUDRATE_DEFAULT, reverse=False, preview=False):
        super().__init__(port,  preview, baudrate)
        self.steps = 0
        self.reverse = reverse
        self.steps_per_unit = steps_per_unit

    def step(self, forward):
        self.steps += 1 if forward else -1
        if self.reverse:
            forward = not forward
        if not self.preview:
            if self.com == "gpio":
                gpio_control_stepper(1, 1) if forward else gpio_control_stepper(1, 0)
            elif self.com == "serial":
                self.write(b"f" if forward else b"r")


class Servo(Module):
    def __init__(self, pulse_min, pulse_max, port, baudrate=BAUDRATE_DEFAULT, preview=False):
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        super().__init__(port, preview, baudrate)
        self.delay_us = 0

    def pulse(self, delay_us):
        try:
            if not self.preview:
                self.write(delay_us.to_bytes(2, byteorder='little'))
        except serial.SerialException as e:
            pass
        else:
            pass


    def fraction(self, f):
        p = int(self.pulse_min + (self.pulse_max - self.pulse_min) * f)
        self.pulse(p)

    def pressed(self, nc=True):
        self.pulse(65535)
        r = self.port.read(1)
        if nc:
            return r == b"1"
        else:
            return r == b"0"


class Action:
    def __iter__(self):
        return [self].__iter__()



class HomingAction(Action):
    def __init__(self, axis, name, pos, feedrate, nc=True):
        self.axis = axis
        self.name = name
        self.pos = np.array(pos)
        self.feedrate = feedrate
        self.nc = nc


class PathAction(Action):
    def __call__(self, t):
        raise NotImplementedError()

    def init(self, pos_start):
        raise NotImplementedError()


class WaitAction(Action):
    def __init__(self, dt):
        self.dt = dt

    def __call__(self, dt):
        return dt <= self.dt


class SequenceAction(Action):
    def __init__(self, *sub_actions):
        self.sub_actions = sub_actions

    def __iter__(self):
        return self.sub_actions.__iter__()


class ServoAction(Action):
    def __init__(self, name, pulse, dt=0.01, wait=1.0):
        self.name = name
        self.pulse = pulse
        self.dt = dt
        self.wait = wait


class Line(PathAction):
    def __init__(self, pos_end, feedrate):
        self.pos_start = np.zeros_like(pos_end)
        self.pos_end = np.array(pos_end)
        self.duration = -1
        self.feedrate = feedrate

    def init(self, pos_start):
        self.pos_start = np.array(pos_start)
        mask_nan = np.isnan(self.pos_end)
        self.pos_end[mask_nan] = self.pos_start[mask_nan]
        self.duration = np.linalg.norm(self.pos_end - self.pos_start) / self.feedrate

    def __call__(self, t):
        if t > self.duration:
            # end move
            return None
        u = t / self.duration
        return self.pos_start * (1 - u) + self.pos_end * u


def transform_corexy(pos, pos_transform):
    pos_transform[:] = pos[:]
    pos_transform[0] = pos[0] + pos[1]
    pos_transform[1] = pos[0] - pos[1]


def modules_manager(action_queue, modules_config, pos_transformer=None, preview=False):
    logging.info("start loop")
    modules = {}

    modules_axis = {}

    for name, config in modules_config.items():
        if config["type"] == "stepper":
            steps_per_unit = config["steps_per_revolutions"] / (config["shaft_diameter"] * math.pi)
            obj = Stepper(steps_per_unit,
                          config["port"],
                          config["baudrate"],
                          reverse=config.get("reverse", False), preview=preview)
            modules[name] = obj
            if "axis" in config:
                modules_axis[config["axis"]] = obj
        elif config["type"] == "servo":
            modules[name] = Servo(config["pulse_min"],
                                  config["pulse_max"],
                                  config["port"],
                                  config["baudrate"], preview=preview)

    n_axis = len(modules_axis)
    pos = np.zeros((n_axis,))
    pos_motors = np.zeros((n_axis,))
    z_up = False
    x_plot_low= []
    y_plot_low = []
    x_plot_high= []
    y_plot_high = []

    def tick_motor():
        if pos_transformer is None:
            pos_motors[:] = pos[:]
        else:
            pos_transformer(pos, pos_motors)
        p0 = np.zeros((n_axis,))
        p1 = np.zeros((n_axis,))
        for j in range(n_axis):
            m = modules_axis[j]
            p0[j] = m.steps/m.steps_per_unit
            s = int(pos_motors[j] * m.steps_per_unit)
            if m.steps < s:
                m.step(True)
            elif m.steps > s:
                m.step(False)
            p1[j] = m.steps/m.steps_per_unit
        x = 0.5*(p1[0] + p1[1])
        y = 0.5*(p1[0] - p1[1])

        if not z_up:
            x_plot_low.append(x)
            y_plot_low.append(y)
        else: 
            x_plot_high.append(x)
            y_plot_high.append(y)
            
            
    started = False
    while True:
        if not action_queue.empty():
            started = True
            action = action_queue.get()
            t0 = time.perf_counter()

            for sub_action in action:
                if isinstance(sub_action, PathAction):
                    # time in s, ~us resolution
                    sub_action.init(pos)

                    while True:
                        t = time.perf_counter()

                        # path is a time function
                        pos_new = sub_action(t - t0)

                        if pos_new is None:
                            # done
                            break

                        pos[:] = pos_new[:]
                        tick_motor()
                elif isinstance(sub_action, WaitAction):
                    dt = 0
                    while sub_action(dt):
                        dt = time.perf_counter() - t0
                elif isinstance(sub_action, ServoAction):
                    print("hello")
                    z_up = not z_up
                    dt1 = 0
                    t0_pwm = t0
                    action_wait = WaitAction(sub_action.wait)
                    while action_wait(dt1):
                        t1 = time.perf_counter()
                        dt1 = t1 - t0
                        dt2 = t1 - t0_pwm
                        if dt2 >= sub_action.dt:
                            t0_pwm = t1
                            modules[sub_action.name].pulse(sub_action.pulse)

                elif isinstance(sub_action, HomingAction):
                    line = Line(sub_action.pos, sub_action.feedrate)
                    line.init(pos)

                    while True:
                        t = time.perf_counter()

                        # path is a time function
                        pos_new = line(t - t0)

                        if pos_new is None:
                            logging.error(f"Homing failed for axis {sub_action.axis}")
                            break

                        pos[:] = pos_new[:]

                        tick_motor()

                        if modules[sub_action.name].pressed(sub_action.nc):
                            logging.info(f"Homing axis {sub_action.axis}")
                            # homing success
                            for i in range(n_axis):
                                motor = modules_axis[i]
                                motor.steps = 0
                            pos[sub_action.axis] = 0
                            break
                     
        elif started and action_queue.empty():
            plt.figure(figsize=(10, 6))  # Set the figure size   
            plt.xlabel('X coordinate (mm)')
            plt.ylabel('Y coordinate (mm)')
            plt.title('Plot of Paths')
            plt.axis('equal')  # Ensure aspect ratio is equal to make plot proportions correct
            plt.axis("off")
            plt.plot(x_plot_low, y_plot_low, linestyle='-', color='b', label='Z_DOWN')
            plt.plot(x_plot_high, y_plot_high, linestyle='-', color='r', label='Z_UP')
            #plt.savefig(os.path.join("examples/out", "logo.png"))
            plt.show()
            
            break

        


def parse_xy(filename, action_queue,
             feedrate,
             servo_up_action=None,
             servo_down_action=None):

   # action_queue.put(homing_action)

    with open(filename, "r") as f:
        for line in f.readlines():
            if line.upper().startswith("UP"):
                if servo_up_action is not None:
                    action_queue.put(servo_up_action)
            elif line.upper().startswith("DOWN"):
                if servo_down_action is not None:
                    action_queue.put(servo_down_action)
            else:
                action_queue.put(Line([float(x) for x in line.strip().split(",")], feedrate))


def parse_svg(filename, action_queue, default_feedrate,  servo_up_action=None, servo_down_action=None, max_width=200, max_height=200):
    feedrate = default_feedrate
    ad = axidraw.AxiDraw()             # Create class instance
    ad.plot_setup(filename)    # Parse the input file
    ad.options.preview  = True
    ad.plot_run()
    scale_factor_width = max_width / ad.digest.width
    scale_factor_height = max_height / ad.digest.height
    scale_factor = min(scale_factor_width, scale_factor_height)
    for layer in ad.digest.layers:
        for path in layer.paths:
            for subpath in path.subpaths:
                for i, (x, y) in enumerate(subpath):
                    x_s, y_s = x*scale_factor, y*scale_factor
                    if i == 0:
                        action_queue.put(servo_up_action)
                        action_queue.put(Line([x_s, y_s], feedrate))
                        action_queue.put(servo_down_action)
                    else:
                        action_queue.put(Line([x_s, y_s], feedrate))

    action_queue.put(servo_up_action)


def parse_arguments():
    usage_text = (
        "Usage:  python urumbu_corexy.py [options]"
    )
    parser = argparse.ArgumentParser(description=usage_text)
    parser.add_argument("-f", "--filename", type=str, required=True,
                        help="filename for .xy file")
    parser.add_argument("--feedrate", type=float, default=5,
                        help="feedrate for XY motion")
    parser.add_argument("--host", type=str, default="laptop")
    parser.add_argument("-a", type=str, default="/dev/ttyACM0",
                        help="COM port for A")
    parser.add_argument("-b", type=str, default="/dev/ttyACM1",
                        help="COM port for B")
    parser.add_argument("-s", default="/dev/ttyACM2",
                    help="COM port for servo")
    parser.add_argument("--max-width", type=int, default=50,
                        help="max widht of the plot in mm") 
    parser.add_argument("--max-height", type=int, default=50,
                        help="max height of the plot in mm")
    parser.add_argument("--preview", type=bool, default=False,
                        help="max height of the plot in mm")
    

    return parser.parse_known_args()

def main():
    multiprocessing.set_start_method('fork')
    action_queue = multiprocessing.Queue()

    args, _ = parse_arguments()
    if args.host=="rpi":
        from gpiozero import OutputDevice
        # multistepping
        gpio2 = OutputDevice(2, initial_value=True)
        gpio3 = OutputDevice(3, initial_value=True)
        gpio4 = OutputDevice(4, initial_value=True)


    modules_config = {
        "a": {
            "type": "stepper",
            "port": "/dev/ttyACM0",
            "baudrate": 921600,
            "axis": 0,
            "shaft_diameter": 19.872,
            "steps_per_revolutions": 16*200,
            "reverse": True
        },
        "b": {
            "type": "stepper",
            "port": args.b,
            "baudrate": 921600,
            "axis": 1,
            "shaft_diameter": 19.872,
            "steps_per_revolutions": 16*200,
            "reverse": True
        },
        "servo": {
            "type": "servo",
            "port": args.s,
            "pulse_min": 600,
            "pulse_max": 2500,
            "baudrate": 921600,
        }
    }
    filename = args.filename
    max_width = args.max_width
    max_height = args.max_height
    preview = args.preview

    p1 = multiprocessing.Process(target=modules_manager, args=(action_queue, modules_config, transform_corexy, preview))
    
    p1.start()
    feedrate = args.feedrate
    #feedrate_homing = 20

    #homing_action = SequenceAction(HomingAction(0, "servo", [-200, 0], feedrate_homing),
     #                              HomingAction(1, "b", [0, -200], feedrate_homing))

    ext = os.path.splitext(filename)[-1]
    if ext == ".xy":
        parse_xy(filename, action_queue, feedrate, servo_up_action=ServoAction("servo", 1265, wait=0.5),
                 servo_down_action=ServoAction("servo", 800, wait=0.5))
    elif ext == ".svg":
        parse_svg(filename, action_queue, feedrate, servo_up_action=ServoAction("servo", 1200, wait=0.5),
                 servo_down_action=ServoAction("servo", 600, wait=0.5), max_width=max_width, max_height=max_height)
    else:
        print(f"Unrecognized file type: '{ext}'")
    print(action_queue.qsize())

if __name__ == "__main__":
    main()
