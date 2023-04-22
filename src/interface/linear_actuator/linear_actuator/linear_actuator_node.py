'''
Package:   linear_actuator
Filename:  linear_actuator_node.py
Author:    Will Heitman (w at heit.mn)

Subscribes to:
- /carla/hero/vehicle_control_cmd (CarlaEgoVehicleControl)
- /guardian/mode (nova_msgs/msg/Mode)

Sends the appropriate brake data to the LA

✨ Documentation available: nova-utd.github.io/interface/linear-actuators
'''

import os
import random
import time

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

# Message definitions
from carla_msgs.msg import CarlaEgoVehicleControl
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nova_msgs.msg import Mode
from rosgraph_msgs.msg import Clock


# bus = None
COMMAND_ID = 0xFF0000
REPORT_ID = 0xFF0001


class linear_actuator_node(Node):
    vehicle_command_sub = None
    brake = None

    def __init__(self):
        super().__init__('linear_actuator_node')

        self.bus = None

        self.vehicle_command_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.sendBrakeControl, 10)

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

        self.retry_connection_timer = self.create_timer(1.0, self.connectToBus)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

        # channel = '/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001C000F4E50430120303838-if00'
        # bitrate = 250000
        # self.bus = can.interface.Bus(bustype='slcan', channel=channel, bitrate=bitrate, receive_own_messages=True)

        # while self.bus is None:
        #     self.get_logger().warn("Bus not yet set. Waiting...")
        #     time.sleep(1.0)
        # self.get_logger().warn("Bus SET.")

        # self.brake_control_timer = self.create_timer(
        # 0.1, self.sendBrakeControl(self.bus))

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

    def connectToBus(self):
        self.bus: can.interface.Bus
        if self.bus is not None and self.bus.state == can.bus.BusState.ACTIVE:
            return
        try:
            self.status = self.initStatusMsg()
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = "Connecting to LA."
            channel = '/dev/serial/by-id/usb-OnLogic_K800_eMCU_50010066514151D2-if02'
            bitrate = 250000
            self.bus = can.interface.Bus(
                bustype='slcan', channel=channel, bitrate=bitrate, )
            self.get_logger().warning("CONNECTED")
            response = self.enableClutch(self.bus)
            self.get_logger().warning(
                f"Clutch enabled with response {response}")

        except can.exceptions.CanInitializationError as e:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = f"LA failed to connect to bus. {e}."
            self.status_pub.publish(self.status)
        return

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def sendBrakeControl(self, msg: CarlaEgoVehicleControl):
        # self.get_logger().info('Printing self.brake in sendBrakeControl')
        # self.get_logger().info(str(msg.brake))
        if self.bus is None:
            self.get_logger().warn("Bus not yet set. Skipping command")
        if msg.brake < 0.1:
            # We don't bother with small brake signals
            return

        position = max((1-msg.brake) / 2 - 0.1, 0.0)

        # self.get_logger().info(f"{msg.brake} => {position}")
        response = self.sendToPosition(position, self.bus)
        # self.get_logger().info(f"Got response: {response}")

    def enableClutch(self, bus: can.Bus):
        clutch_enable = True
        motor_enable = False
        clutch_enable_byte = clutch_enable * 0x80
        motor_enable_byte = motor_enable * 0x40
        byte3 = clutch_enable_byte + motor_enable_byte

        data = [0x0F, 0x4A, 0, byte3, 0, 0, 0]

        message = can.Message(
            arbitration_id=COMMAND_ID, data=data, is_extended_id=True)

        bus.send(message)

        msg = bus.recv(0.1)
        return msg

    def disableClutch(self):
        """Turn off the motor, then the clutch.

        Args:
            bus (can.Bus): Bus to interact with

        Returns:
            _type_: _description_
        """

        bus = self.bus

        # Disable motor
        clutch_enable = True
        motor_enable = False
        clutch_enable_byte = clutch_enable * 0x80
        motor_enable_byte = motor_enable * 0x40
        byte3 = clutch_enable_byte + motor_enable_byte

        data = [0x0F, 0x4A, 0, byte3, 0, 0, 0]

        message = can.Message(
            arbitration_id=COMMAND_ID, data=data, is_extended_id=True)

        bus.send(message)

        time.sleep(0.1)  # Give motor time to stop

        # Disable clutch
        clutch_enable = False
        motor_enable = False
        clutch_enable_byte = clutch_enable * 0x80
        motor_enable_byte = motor_enable * 0x40
        byte3 = clutch_enable_byte + motor_enable_byte

        data = [0x0F, 0x4A, 0, byte3, 0, 0, 0]

        message = can.Message(
            arbitration_id=COMMAND_ID, data=data, is_extended_id=True)

        bus.send(message)

        msg = bus.recv(0.1)
        return msg

    def sendToPosition(self, pos: float, bus: can.Bus):

        # self.get_logger().info('hey im in the sendtoposition function')
        """Given position, send appropriate CAN messages to LA

        Args:
            pos (float): Position from 0.0 (fully extended) to 1.0 (fully retracted)

        Position command format:
        [0x0F 0x4A  DPOS_LOW Byte3 0 0 0 0]

        Byte 3:
        [ClutchEnable MotorEnable POS7 POS6 POS5 POS4 POS3]
        """

        self.status = self.initStatusMsg()

        if self.bus is None:
            self.get_logger().warn("Bus not yet set.")
            return

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

        try:
            bus.send(message)
            print("Sending message!")
            msg = bus.recv(0.1)
            print(f"Got response {msg}")
        except can.exceptions.CanError as e:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = "LA failed to send command: {e}."

        self.status_pub.publish(self.status)

        return msg


def main(args=None):
    rclpy.init(args=args)

    LAN = linear_actuator_node()

    rclpy.spin(LAN)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    linear_actuator_node.disableClutch()
    linear_actuator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()