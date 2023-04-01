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


class GnssInterfaceNode(Node):

    def __init__(self):
        super().__init__('gnss_interface_node')

        
        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)
        
        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)
        
        self.retry_connection_timer = self.create_timer(1.0, self.connectToPort)

    def connectToPort(self):
        self.bus: can.interface.Bus
        if self.bus is not None and self.bus.state == can.bus.BusState.ACTIVE:
            return
        try:
            self.bus = can.interface.Bus(
                bustype='slcan', channel='/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001500174E50430520303838-if00', bitrate=500000, receive_own_messages=True)
        except can.exceptions.CanInitializationError as e:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = f"EPAS failed to connect to bus: {e}"
            self.status_pub.publish(self.status)
        return
        
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

    


def main(args=None):
    rclpy.init(args=args)
    gnss_interface_node = GnssInterfaceNode()
    rclpy.spin(gnss_interface_node)
    gnss_interface_node.destroy_node()
    rclpy.shutdown()
