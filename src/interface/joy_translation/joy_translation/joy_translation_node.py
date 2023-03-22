'''
Package:   joy_translation
Filename:  joy_translation_node.py
Author:    Ashwin Somasundaram, Will Heitman (w at heit.mn)

Converts Joy messages to vehicle control commands

Xbox One controller axes:
[0]: LS, X
[1]: LS, Y
[2]: LT
[3]: LS, X
[4]: LS, Y
[5]: RT
[6]: Dpad X
[7]: Dpad Y

Xbox One controller buttons:
[0]: A
[1]: B
[2]: X
[3]: Y
[4]: LB
[5]: RB
[6]: Select (left)
[7]: Start (right)
[8]: Xbox
[9]: LSB
[10]: RSB

Controls:
A: Auto enable (hold)
X: Manual enable (hold, takes precedence over A)
LS: Steering position
LT: Brake
RT: Throttle

Subscribes to:
 /joy (sensor_msgs/Joy)

Publishes to:
/carla/hero/vehicle_control_cmd (CarlaEgoVehicleControl)
'''

import numpy as np
from rosgraph_msgs.msg import Clock

from carla_msgs.msg import CarlaEgoVehicleControl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class joy_translation_node(Node):
    joy_sub = 0.0
    command_pub = None

    def __init__(self):
        super().__init__('joy_translation_node')
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joyCb, 10)
        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)

    def joyCb(self, joy_msg: Joy):
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = Clock().clock
        msg.header.frame_id = 'base_link'
        msg.throttle = ((joy_msg.axes[5]*-1)+1)/2

        # TODO: Fix this jank.
        if msg.throttle == 0.5:
            msg.throttle = 0.0

        msg.steer = joy_msg.axes[0]*-1
        msg.brake = joy_msg.axes[2]*-1

        # Let's leave these out for now, as Navigator does not support them.
        # msg.hand_brake = True if joy_msg.buttons[2] == 1 else False
        # msg.reverse = True if joy_msg.buttons[0] == 1 else False
        # msg.gear = True if joy_msg.buttons[3] == 1 else False
        # msg.manual_gear_shift = True if joy_msg.buttons[1] == 1 else False

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    JOYNODE = joy_translation_node()
    rclpy.spin(JOYNODE)
    # self.get_logger().info('#####################################################################################')
    joy_translation_node.destroy_node()
    rclpy.shutdown()
