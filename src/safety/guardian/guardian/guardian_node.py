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
from nova_msgs.msg import Mode
import rclpy
from rclpy.node import Node


class guardian_node(Node):
    joy_sub = 0.0
    command_pub = None
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('guardian_node')

        self.mode_request_sub = self.create_subscription(
            Mode, '/requested_mode', self.modeRequestCb, 1)

        self.current_mode_pub = self.create_publisher(
            Mode, '/guardian/mode', 1)

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
