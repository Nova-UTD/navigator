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
import time

from carla_msgs.msg import CarlaEgoVehicleControl
from navigator_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from navigator_msgs.msg import Mode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class joy_translation_node(Node):
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('joy_translation_node')

        self.current_speed = 0.0  # m/s

        joy_sub = self.create_subscription(
            Joy, '/joy', self.joyCb, 10)

        self.clock = Clock().clock
        clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)

        self.requested_mode_pub = self.create_publisher(
            Mode, '/requested_mode', 1)

        speed_sub = self.create_subscription(
            CarlaSpeedometer, '/speed', self.speedCb, 1)

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

    def speedCb(self, msg: CarlaSpeedometer):
        self.current_speed = msg.speed

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def initStatusMsg(self) -> DiagnosticStatus:
        status = DiagnosticStatus()

        status.name = self.get_name()

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)
        status.values.append(stamp)

        status.level = DiagnosticStatus.OK

        return status

    def getSpeedAdjustedSteering(self, joystick_pos: float) -> float:
        """When the car is moving, we want to increase the steering sensitity of our
        joystick by tightening its bounds, reducing the maximum steering in either direction.

        Args:
            joystick_pos (float): _description_

        Returns:
            float: _description_
        """
        # We should never reduce steering more than this.
        # Dampening is proportional to this
        MIN_STEERING_LIMIT = 0.5
        MAX_SPEED = 10  # m/s. Descriptive, not prescriptive.

        # Just an unconstrained, parabolic map.
        parabolic_steer = joystick_pos**2
        if joystick_pos < 0:
            parabolic_steer *= -1

        # "Squish" this curve vertically based on current speed
        tightener = -0.05 * self.current_speed + 1
        steer = parabolic_steer * tightener

        return steer

    def joyCb(self, msg: Joy):
        command_msg = CarlaEgoVehicleControl()

        self.status = self.initStatusMsg()

        left_trigger = msg.axes[2]
        # if left_trigger == 0.:
        #     left_trigger = -1.
        right_trigger = msg.axes[5]
        if right_trigger == 0.:
            right_trigger = 1.

        command_msg.header.stamp = self.clock
        command_msg.header.frame_id = 'base_link'
        command_msg.throttle = ((right_trigger*-1)+1)/2
        # self.get_logger().info(str(command_msg.throttle))
        # command_msg.throttle = 0.0
        command_msg.steer = self.getSpeedAdjustedSteering(msg.axes[0]*-1)

        if left_trigger == 0.:
            command_msg.brake = 0.
        else:
            command_msg.brake = 1-(left_trigger+1)/2
        # command_msg.brake = 1.0

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
            # self.get_logger().info("Publishing manual command!")
        self.requested_mode_pub.publish(requested_mode)

        requested_mode_keyval = KeyValue()
        requested_mode_keyval.key = 'requested_mode'
        requested_mode_keyval.value = str(requested_mode)
        self.status.values.append(requested_mode_keyval)
        self.status_pub.publish(self.status)


def main(args=None):
    rclpy.init(args=args)
    joy_translator = joy_translation_node()
    rclpy.spin(joy_translator)

    joy_translation_node.destroy_node()
    rclpy.shutdown()
