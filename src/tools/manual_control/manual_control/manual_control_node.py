#!/usr/bin/python

'''
Nova at UT Dallas, 2022

Accepts control commands from humans.

Currently targets keyboard control.
Will target joystick control using joy_node in the future.
'''

# For fancy console UI stuff
import curses
from pynput import keyboard

import random
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener
import math
from os.path import exists

# Message definitons
from std_msgs.msg import Header
from sensor_msgs.msg import Image # For cameras
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry # For GPS, ground truth
from voltron_msgs.msg import PeddlePosition, SteeringPosition
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2

screen = curses.initscr()
num_rows, num_cols = screen.getmaxyx()

pressed_keys = []

def on_kb_update():
    # print(pressed_keys)
    if 'w' in pressed_keys:
        screen.addstr(6,5,"⬆", curses.A_REVERSE)
        screen.refresh()
    else:
        screen.addstr(6,5,"⬆", curses.A_NORMAL)
        screen.refresh()
    if 's' in pressed_keys:
        screen.addstr(8,5,"⬇", curses.A_REVERSE)
        screen.refresh()
    else:
        screen.addstr(8,5,"⬇", curses.A_NORMAL)
        screen.refresh()
    if 'a' in pressed_keys:
        screen.addstr(7,3,"⬅", curses.A_REVERSE)
        screen.refresh()
    else:
        screen.addstr(7,3,"⬅", curses.A_NORMAL)
        screen.refresh()
    


def on_press(key):
    try:
        if key.char == 'q':
            screen.nodelay(False)
            curses.nocbreak()
            screen.keypad(False)
            curses.endwin()
            rclpy.shutdown()
        if pressed_keys.count(key.char) == 0:
            pressed_keys.append(key.char)
            on_kb_update()
        # print(pressed_keys)

    except AttributeError:
        return
        # print('special key {0} pressed'.format(
        #     key))

def on_release(key):
    try: 
        pressed_keys.remove(key.char)
        on_kb_update()
    except:
        return
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def print_center(message):
    # Calculate center row
    middle_row = int(num_rows / 2)

    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message

    screen.addstr(0 ,0,'N════════════════════════O')
    screen.addstr(1 ,0,'║      MANUAL CONTROL    ║')
    screen.addstr(2 ,0,'╠════════════════════════╣')
    screen.addstr(3 ,0,'║ WASD,Shift,R           ║')
    screen.addstr(4 ,0,'╠════════════════════════╣')
    screen.addstr(5 ,0,'║ ┌─────┐  T ███████████ ║')
    screen.addstr(6 ,0,'║ │  ⬆  │  B █████▒▒▒▒▒▒ ║')
    screen.addstr(7 ,0,'║ │⬅ B ➡│  S ▒▒▒▒▒:▒▒▒▒▒ ║')
    screen.addstr(8 ,0,'║ │  ⬇  │                ║')
    screen.addstr(9 ,0,'║ └─────┘                ║')
    screen.addstr(10,0,'║         Press q to quit║')
    screen.addstr(11,0,'V════════════════════════A')
    curses.echo(False)
    screen.refresh()

    screen.nodelay(True)
    curses.cbreak()
    screen.keypad(1)
    # curses.endwin()
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

# ╔═══════════════╗
# ║ MANUAL CONTROL║
# ╠═══════════════╣
# ║ WASD,Shift,R  ║
# ╠═══════════════╣
# ║ T ███████████ ║
# ║ B █████▒▒▒▒▒▒ ║
# ║ S ▒▒▒▒▒:▒▒▒▒▒ ║
# ╚═══════════════╝

class ManualControlNode(Node):

    def __init__(self):
        super().__init__('manual_control_node')

        # Create our publishers
        self.steering_command_pub = self.create_publisher(
            SteeringPosition,
            '/command/steering_position',
            10
        )

        self.throttle_command_pub = self.create_publisher(
            PeddlePosition,
            '/command/throttle_position',
            10
        )

        self.brake_command_pub = self.create_publisher(
            PeddlePosition,
            '/command/brake_position',
            10
        )

        print("Trying wrapper...")
        print_center("Hello world!")
        

def main(args=None):
    rclpy.init(args=args)

    manual_control_node = ManualControlNode()

    rclpy.spin(manual_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manual_control_node.destroy_node()
    curses.endwin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()