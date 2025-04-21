"""
Initial node for state estimation

haven't yet added IMU data.
"""

import math

import rclpy
import numpy as np

from numpy import typing as npt

from rclpy.node import Node

from std_msgs.msg import ColorRGBA, Header
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker

from navigator_msgs.msg import VehicleControl, VehicleSpeed

import io
from dataclasses import dataclass
from datetime import datetime

# Message definitions
from navigator_msgs.msg import VehicleSpeed 
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster

from shapely.geometry import LineString, Point
import shapely



class StateEstimationNode(Node):
    def __init__(self):
        super().__init__('state_estimation_node')
        # TF broadcaster for fused state estimates
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to GNSS odometry (smoothed)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gnss/odometry',  # can be remapped via ROS2 launch
            self.odom_callback,
            10
        )

        # In the future, add IMU subscription here for fusion
        # self.imu_sub = self.create_subscription(
        #     Imu,
        #     '/carla/hero/imu',
        #     self.imu_callback,
        #     10
        # )

    def odom_callback(self, msg: Odometry):
        # Build TransformStamped from state estimate
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Use GNSS position for translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Use GNSS orientation (from IMU fusion later)
        # t.transform.rotation = msg.pose.pose.orientation


        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
