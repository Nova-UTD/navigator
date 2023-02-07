'''
Package: state_estimation
   File: gnss_averaging_node.py
 Author: Will Heitman (w at heit dot mn)

Very simple node to convert raw GNSS odometry into a map->base_link transform.
'''

import math
import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from builtin_interfaces.msg import Time

from carla_msgs.msg import CarlaSpeedometer, CarlaWorldInfo
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix

from tf2_ros import TransformBroadcaster
from xml.etree import ElementTree as ET


class GnssAveragingNode(Node):

    def __init__(self):
        super().__init__('gnss_estimation_node')

        self.raw_gnss_sub = self.create_subscription(
            Odometry, '/odometry/gnss_raw', self.raw_gnss_cb, 10
        )

        self.speedometer_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedometer_cb, 10)

    def raw_gnss_cb(self, msg: Odometry):
        self.get_logger().info("Got GNSS!")

    def speedometer_cb(self, msg: CarlaSpeedometer):
        self.get_logger().info(f"Got speed {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    gnss_averaging_node = GnssAveragingNode()

    rclpy.spin(gnss_averaging_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gnss_averaging_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
