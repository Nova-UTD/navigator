'''
Package:   linear_actuator
Filename:  linear_actuator_node.py
Author:    Will Heitman (w at heit.mn)

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

✨ Documentation available: nova-utd.github.io/interface/linear-actuators
'''

import numpy as np
from rosgraph_msgs.msg import Clock
import time

from carla_msgs.msg import CarlaEgoVehicleControl
from nova_msgs.msg import Mode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class joy_translation_node(Node):
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('joy_translation_node')
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joyCb, 10)

        self.clock = Clock().clock

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)

        self.requested_mode_pub = self.create_publisher(
            Mode, '/requested_mode', 1)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def joyCb(self, msg: Joy):
        command_msg = CarlaEgoVehicleControl()

        command_msg.header.stamp = self.clock
        command_msg.header.frame_id = 'base_link'
        command_msg.throttle = ((msg.axes[5]*-1)+1)/2

        # TODO: Fix this jank.
        if command_msg.throttle == 0.5:
            command_msg.throttle = 0.0

        command_msg.steer = msg.axes[0]*-1
        command_msg.brake = 1-(msg.axes[2]+1)/2

        requested_mode = Mode()
        if msg.buttons[2] == 1:
            requested_mode.mode = Mode.MANUAL
        elif msg.buttons[0] == 1:
            requested_mode.mode = Mode.AUTO
        else:
            requested_mode.mode = Mode.DISABLED

        # Let's leave these out for now, as Navigator does not support them.
        # msg.hand_brake = True if joy_msg.buttons[2] == 1 else False
        # msg.reverse = True if joy_msg.buttons[0] == 1 else False
        # msg.gear = True if joy_msg.buttons[3] == 1 else False
        # msg.manual_gear_shift = True if joy_msg.buttons[1] == 1 else False

        if self.current_mode == Mode.MANUAL:
            self.command_pub.publish(command_msg)
            self.get_logger().info("Publishing manual command!")
        self.requested_mode_pub.publish(requested_mode)


def main(args=None):
    rclpy.init(args=args)
    joy_translator = joy_translation_node()
    rclpy.spin(joy_translator)

    joy_translation_node.destroy_node()
    rclpy.shutdown()
