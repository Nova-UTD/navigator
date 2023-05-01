import can
from can.bus import BusState
import math

from carla_msgs.msg import CarlaEgoVehicleControl
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nova_msgs.msg import Mode
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from dataclasses import dataclass
from std_msgs.msg import Float32
import numpy as np


@dataclass
class EpasState():
    duty: int
    current: int
    supply_voltage_mv: int
    temperature: int
    angle: int
    errors: int


class EpasNode(Node):

    def __init__(self):
        super().__init__('epas_node')

        self.limit_left: int = 19
        self.limit_right: int = 230

        self.bus = None

        self.past_errors = np.zeros(5)

        # try:
        #     self.bus = can.interface.Bus(
        #         bustype='slcan', channel='/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001500174E50430520303838-if00', bitrate=500000, receive_own_messages=True)
        # except can.exceptions.CanInitializationError:

        self.command_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.commandCb, 1)
        self.cmd_msg = None
        self.cmd_timer = self.create_timer(.01, self.vehicleControlCb)
        self.current_angle = 0.0

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)
        self.target_angle = 0.0
        self.cached_msg1 = None

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)
        
        # self.current_angle_pub = self.create_publisher(Float32, )

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

        self.retry_connection_timer = self.create_timer(1.0, self.connectToBus)

    def connectToBus(self):
        self.bus: can.interface.Bus
        if self.bus is not None and self.bus.state == can.bus.BusState.ACTIVE:
            return
        try:
            self.bus = can.interface.Bus(
                bustype='slcan', channel='/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001500174E50430520303838-if00', bitrate=500000, receive_own_messages=True)
        except can.exceptions.CanInitializationError as e:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = f"EPAS failed to connect to bus. {e}"
            self.status_pub.publish(self.status)
        return

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def initStatusMsg(self) -> DiagnosticStatus:
        status = DiagnosticStatus()

        status.name = self.get_name()

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)
        status.values.append(stamp)

        status.level = DiagnosticStatus.OK

        return status

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def commandCb(self, msg: CarlaEgoVehicleControl):
        self.cmd_msg = msg

    def parseIncomingMessages(self, msg1_data: bytearray, msg2_data: bytearray) -> EpasState:
        torque = msg1_data[0]
        duty = msg1_data[1]
        current = msg1_data[2]
        supply_voltage = msg1_data[3]
        switch_pos = msg1_data[4]
        temp = msg1_data[5]
        torque_a = msg1_data[6]
        torque_b = msg1_data[7]

        angle = msg2_data[0]
        analog_c1 = msg2_data[1]
        analog_c2 = msg2_data[2]
        selected_map = msg2_data[3]
        errors = msg2_data[4]
        dio_bitfield = msg2_data[5]
        status_bitfield = msg2_data[6]
        limit_bitfield = msg2_data[7]

        current_state = EpasState(
            duty, current, supply_voltage, temp, angle, errors)

        return current_state

    def sendCommand(self, target, bus):
        current_angle_normalized = (
            (self.current_angle-self.limit_left)/(self.limit_right-self.limit_left)*2)-1
        # self.get_logger().info('current angle'+str(current_angle_normalized))
        e = target - current_angle_normalized  # Error = target - current
        # self.get_logger().info('current_angle_normalized: '+str(current_angle_normalized))
        # self.get_logger().info('target: '+str(target))

        # We need to map [-1.0, 1.0] to [0, 255
        
        # Append new incoming error
        np.roll(self.past_errors, -1)
        self.past_errors[-1] = e
        

        Kp = .4
        Ki = .68

        INTEGRAL_CAP = .08
        TORQUE_LIMIT = 200

        integral_term = np.sum(self.past_errors) * Ki

        if(integral_term > INTEGRAL_CAP):
            integral_term = INTEGRAL_CAP
        elif(integral_term < -1 * INTEGRAL_CAP):
            integral_term = -1 * INTEGRAL_CAP

        # Power is an abstract value from [-1., 1.], where -1 is hard push left
        power = e * Kp + integral_term

        torqueA: int = min(TORQUE_LIMIT, max(0, math.ceil((power+1) * (255/2))))
        torqueB: int = 255-torqueA
        # self.get_logger().info(str(torqueA))

        data = [0x03, torqueA, torqueB, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x296, data=data,
                              check=True, is_extended_id=False)
        bus.send(message, timeout=0.2)

    def vehicleControlCb(self):
        self.status = self.initStatusMsg()

        if self.bus is None or self.bus.state == BusState.ERROR:
            # Status already generated by connectToBus

            # self.status.level = DiagnosticStatus.ERROR
            # self.status.message = "CAN bus was in error state."
            # self.status_pub.publish(self.status)
            return
        elif self.cmd_msg == None:
            self.status.level = DiagnosticStatus.WARN
            self.status.message = "EPAS command message not received."
            self.status_pub.publish(self.status)
            return

        if self.cmd_msg != None:
            self.target_angle = self.cmd_msg.steer

        response_msg = self.bus.recv(0.1)
        if (response_msg is None):
            self.get_logger().error("No message received")
        elif (response_msg.arbitration_id == 0x290):
            self.cached_msg1 = response_msg.data
        elif (response_msg.arbitration_id == 0x292):
            if (self.cached_msg1 is None):
                return
            msg1 = self.cached_msg1
            msg2 = response_msg.data
            current_state = self.parseIncomingMessages(msg1, msg2)
            self.current_angle = current_state.angle

        if self.current_mode == Mode.MANUAL or \
                self.current_mode == Mode.AUTO:
            self.sendCommand(self.target_angle, self.bus)
        self.status_pub.publish(self.status)


def main(args=None):
    rclpy.init(args=args)
    epas_node = EpasNode()
    rclpy.spin(epas_node)
    epas_node.destroy_node()
    rclpy.shutdown()