'''
Package:   guardian
Filename:  guardian_node.py
Author:    Will Heitman (w at heit.mn)

The Guardian node aggregates Navigator's diagnostic data and publishes:
- Global state description ("Stopping at intersection"),
- Mode (Enabled, Manual, Disabled),
- and global status level (OK, WARN, or ERROR).

Subscribes to:
 /node_statuses (diagnostic_msgs/DiagnosticStatus)

Publishes to:
/status (diagnostic_msgs/DiagnosticArray)
'''

import numpy as np
from rosgraph_msgs.msg import Clock

from carla_msgs.msg import CarlaEgoVehicleControl
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from nova_msgs.msg import Mode
import rclpy
from rclpy.node import Node
from dataclasses import dataclass


class Sensitivity:
    PREVENTS_AUTO = 0
    PREVENTS_MANUAL = 1
    PREVENTS_NOTHING = 2


@dataclass
class StatusEntry:
    level: DiagnosticStatus.level
    message: str = ""


class guardian_node(Node):
    joy_sub = 0.0
    command_pub = None
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('guardian_node')

        self.clock = Clock().clock

        mode_request_sub = self.create_subscription(
            Mode, '/requested_mode', self.modeRequestCb, 1)

        status_sub = self.create_subscription(
            DiagnosticStatus, '/node_statuses', self.statusCb, 10)

        self.status_array_pub = self.create_publisher(
            DiagnosticArray, '/status', 1)

        self.current_mode_pub = self.create_publisher(
            Mode, '/guardian/mode', 1)

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)

        self.watchlist = {
            "rtp_node": ""
        }

        status_timer = self.create_timer(0.5, self.publishStatusArray)

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def initStatusMsg(self, name: str) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)
        status.values.append(stamp)
        return status

    def publishStatusArray(self):

        global_status = self.initStatusMsg('global')
        array_msg = DiagnosticArray()
        array_msg.header.stamp = self.clock

        for entry in self.watchlist:
            status_msg = DiagnosticStatus()
            status_msg.name = entry
            value: StatusEntry = self.watchlist[entry]
            status_msg.level = value.level

            if value.level == DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = value.message
            elif value.level == DiagnosticStatus.WARN and global_status.level != DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.WARN
                global_status.message = value.message

            status_msg.message = value.message
            array_msg.status.append(status_msg)

        array_msg.status.append(global_status)

        self.status_array_pub.publish(array_msg)

    def statusCb(self, msg: DiagnosticStatus):
        if not msg.name in self.watchlist:
            self.get_logger().warning(
                f"Received status from node outside watchlist: {msg.name}")
            return
        self.watchlist[msg.name] = StatusEntry(msg.level, msg.message)

        # print(self.watchlist[msg.name])

    def modeRequestCb(self, msg: Mode):
        self.current_mode = msg.mode

        # Here we can choose to accept or deny the requested mode.

        mode_msg = Mode()
        mode_msg.mode = self.current_mode
        self.current_mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    guardian = guardian_node()
    rclpy.spin(guardian)
    guardian_node.destroy_node()
    rclpy.shutdown()
