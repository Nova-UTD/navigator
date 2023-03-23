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


class Mode:
    DISABLED = 0
    MANUAL = 1
    AUTO = 2


class joy_translation_node(Node):
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('joy_translation_node')
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joyCb, 10)
        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)

    def joyCb(self, msg: Joy):
        command_msg = CarlaEgoVehicleControl()
        command_msg.header.stamp = Clock().clock
        command_msg.header.frame_id = 'base_link'
        command_msg.throttle = ((msg.axes[5]*-1)+1)/2

        # TODO: Fix this jank.
        if command_msg.throttle == 0.5:
            command_msg.throttle = 0.0

        command_msg.steer = msg.axes[0]*-1
        command_msg.brake = msg.axes[2]*-1

        previous_mode = self.current_mode
        if msg.buttons[2] == 1:
            self.current_mode = Mode.MANUAL
        elif msg.buttons[0] == 1:
            self.current_mode = Mode.AUTO
        else:
            self.current_mode = Mode.DISABLED

        if self.current_mode != previous_mode:
            print(f"Mode changed! Is now {self.current_mode}")

        # Let's leave these out for now, as Navigator does not support them.
        # msg.hand_brake = True if joy_msg.buttons[2] == 1 else False
        # msg.reverse = True if joy_msg.buttons[0] == 1 else False
        # msg.gear = True if joy_msg.buttons[3] == 1 else False
        # msg.manual_gear_shift = True if joy_msg.buttons[1] == 1 else False

        self.command_pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)
    JOYNODE = joy_translation_node()
    rclpy.spin(JOYNODE)
    # self.get_logger().info('#####################################################################################')
    joy_translation_node.destroy_node()
    rclpy.shutdown()
