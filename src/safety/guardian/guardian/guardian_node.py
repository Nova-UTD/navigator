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
from nova_msgs.srv import ChangeMode
import rclpy
from rclpy.node import Node


class Mode:
    DISABLED = 0
    MANUAL = 1
    AUTO = 2


class guardian_node(Node):
    joy_sub = 0.0
    command_pub = None
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('guardian_node')

        self.srv = self.create_service(
            ChangeMode, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.success = True
        self.get_logger().info(f"Received: {request}")

        return response

    def changeModeCb(self, request: ChangeMode.Request, response: ChangeMode.Response):
        self.get_logger().info(
            f"Received request to change mode to {request.mode}")

        response.success = True
        response.message = "Done"  # Empty string if successful

        self.get_logger().info(
            f"Exiting service call with response {response}")

        return response


def main(args=None):
    rclpy.init(args=args)
    guardian = guardian_node()
    rclpy.spin(guardian)
    guardian_node.destroy_node()
    rclpy.shutdown()
