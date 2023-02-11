'''
Package:   mcu_interface
Filename:  mcu_interface_node.py
Author:    Will Heitman (w at heit.mn)

Subscribes to CarlaEgoVehicleControl messages (https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaEgoVehicleControl.msg)

Sends the appropriate brake data to the LA
'''

import os
import random
import time

import can
import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

bus = None
COMMAND_ID = 0xFF0000
REPORT_ID = 0xFF0001


class McuInterfaceNode(Node):

    def __init__(self, bus):
        super().__init__('mcu_interface_node')

        self.vehicle_command_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.commandCb, 1)

        self.brake = 0.0  # 0.0 is released, 1.0 is fully pressed

        while bus is None:
            self.get_logger().warn("Bus not yet set. Waiting...")
            time.sleep(1.0)

        self.get_logger().info("Bus now connected.")

    def commandCb(self, msg: CarlaEgoVehicleControl):
        throttle = msg.throttle
        print(f"Throttle = {throttle}")

    def sendBrakeControl(self):
        if bus is None:
            self.get_logger().warn("Bus not yet set. Skipping command")

        response = self.sendToPosition(self.brake, bus)
        print(response)

    def enableClutch(self, bus: can.Bus):
        clutch_enable = True
        motor_enable = False
        clutch_enable_byte = clutch_enable * 0x80
        motor_enable_byte = motor_enable * 0x40
        byte3 = clutch_enable_byte + motor_enable_byte

        data = [0x0F, 0x4A, 0, byte3, 0, 0, 0]

        message = can.Message(
            arbitration_id=COMMAND_ID, data=data, is_extended_id=True)

        msg = bus.recv(0.1)
        return msg

    def sendToPosition(self, pos: float, bus: can.Bus):
        """Given position, send appropriate CAN messages to LA

        Args:
            pos (float): Position from 0.0 (fully extended) to 1.0 (fully retracted)

        Position command format:
        [0x0F 0x4A  DPOS_LOW Byte3 0 0 0 0]

        Byte 3:
        [ClutchEnable MotorEnable POS7 POS6 POS5 POS4 POS3]
        """

        POSITION_MAX = 3.45
        POSITION_MIN = 0.80
        range = POSITION_MAX - POSITION_MIN
        pos_inches = pos * range + POSITION_MIN

        # Account for 0.5 inch offset
        pos_value = int(pos_inches * 1000) + 500

        dpos_hi = int(pos_value / 0x100)
        dpos_low = pos_value % 0x100

        clutch_enable = True
        motor_enable = True
        clutch_enable_byte = clutch_enable * 0x80
        motor_enable_byte = motor_enable * 0x40
        byte3 = sum([dpos_hi, clutch_enable_byte, motor_enable_byte])

        data = [0x0F, 0x4A, dpos_low, byte3, 0, 0, 0]

        message = can.Message(
            arbitration_id=COMMAND_ID, data=data, is_extended_id=True)

        bus.send(message)

        msg = bus.recv(0.1)

        return msg


def main(args=None):
    rclpy.init(args=args)

    # Try ls /dev/seria/by-id
    channel = '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Grand_Central_M4_Express_BA3E99D033533853202020350D3B12FF-if00'
    bitrate = 25000
    bus = can.interface.Bus(
        bustype='slcan', channel=channel, bitrate=bitrate, receive_own_messages=True)

    mcu_interface_node = McuInterfaceNode(bus)

    rclpy.spin(mcu_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mcu_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
