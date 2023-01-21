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
from std_msgs.msg import Bool # This is a no-no, but we're in a hurry. WSH.
from std_msgs.msg import Header
from sensor_msgs.msg import Image # For cameras
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry # For GPS, ground truth
from nova_msgs.msg import PeddlePosition, SteeringPosition
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2

screen = curses.initscr()
num_rows, num_cols = screen.getmaxyx()

pressed_keys = []

def on_kb_update():
    
    reverse = False

    if 'w' in pressed_keys: # Going forward
        screen.addstr(6,5,"⬆", curses.A_REVERSE) # Highlight icon
        screen.addstr(8,5,"⬇", curses.A_NORMAL)
        throttle = 0.7
        reverse = False
    elif 'x' in pressed_keys: # Going in reverse
        screen.addstr(8,5,"⬇", curses.A_REVERSE)
        screen.addstr(6,5,"⬆", curses.A_NORMAL)
        throttle = 1.0
        reverse = True
    else: # Neither forward nor reverse
        screen.addstr(6,5,"⬆", curses.A_NORMAL) # Reset icons
        screen.addstr(8,5,"⬇", curses.A_NORMAL)
        throttle = 0.0
    if 'a' in pressed_keys and 'd' in pressed_keys: # Both left and right, no steer
        screen.addstr(7,3,"⬅", curses.A_REVERSE)
        screen.addstr(7,7,"➡", curses.A_REVERSE)
        steer = 0.0
    elif 'a' in pressed_keys:
        screen.addstr(7,3,"⬅", curses.A_REVERSE)
        screen.addstr(7,7,"➡", curses.A_NORMAL)
        steer = -0.35
    elif 'd' in pressed_keys:
        screen.addstr(7,7,"➡", curses.A_REVERSE)
        screen.addstr(7,3,"⬅", curses.A_NORMAL)
        steer = 0.35
    else:
        screen.addstr(7,7,"➡", curses.A_NORMAL)
        screen.addstr(7,3,"⬅", curses.A_NORMAL)
        steer = 0.0
    if 's' in pressed_keys:
        screen.addstr(7,5,"B", curses.A_REVERSE)
        brake = 0.6
    else:
        screen.addstr(7,5,"B", curses.A_NORMAL)
        brake = 0.0

    # Form a CARLA control command
    throttle_msg = PeddlePosition(data=throttle)
    brake_msg = PeddlePosition(data=brake)
    steer_msg = SteeringPosition(data=steer)
    reverse_msg = Bool(data=reverse)
    throttle_command_pub.publish(throttle_msg)
    brake_command_pub.publish(brake_msg)
    steering_command_pub.publish(steer_msg)
    reverse_command_pub.publish(reverse_msg)

    screen.move(12,24)
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

def handle_ui():

    screen.addstr(0 ,0,'N════════════════════════O')
    screen.addstr(1 ,0,'║      MANUAL CONTROL    ║')
    screen.addstr(2 ,0,'╠════════════════════════╣')
    screen.addstr(3 ,0,'║ WASDX                  ║')
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

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        global steering_command_pub
        global throttle_command_pub
        global brake_command_pub
        global reverse_command_pub

        # Create our publishers
        steering_command_pub = self.create_publisher(
            SteeringPosition,
            '/command/steering_position',
            10
        )

        throttle_command_pub = self.create_publisher(
            PeddlePosition,
            '/command/throttle_position',
            10
        )

        brake_command_pub = self.create_publisher(
            PeddlePosition,
            '/command/brake_position',
            10
        )

        reverse_command_pub = self.create_publisher(
            Bool,
            '/command/reverse',
            10
        )

        handle_ui()

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