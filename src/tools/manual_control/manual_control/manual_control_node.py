#!/usr/bin/python

'''
Nova at UT Dallas, 2022

Accepts control commands from humans.

Currently targets keyboard control.
Will target joystick control using joy_node in the future.
'''

# For fancy console UI stuff
import curses

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

def print_center(message):
    # Calculate center row
    middle_row = int(num_rows / 2)

    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message

    # Draw the text
    screen.addstr(0,0,'Nova Manual Control')
    screen.addstr(1,0,'---------------------')
    screen.addstr(2,0,'W+S: Throttle + brake')
    screen.addstr(3,0,'A+D: Left + right')
    screen.addstr(4,0,'---------------------')
    screen.refresh()

    screen.nodelay(1)
    while True:
        # get keyboard input, returns -1 if none available
        c = screen.getch()
        if c != -1:
            # print numeric value
            screen.addstr(str(c) + ' ')
            screen.refresh()
            # return curser to start position
            screen.move(0, 0)

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
        curses.napms(3000)
        

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