#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2023 - https://github.com/Nova-UTD/navigator/
# Daniel Vayman - Daniel.Vayman@utdallas.edu

from typing import List
from navigator_msgs.msg import Trajectory, TrajectoryPoint, PeddlePosition, SteeringPosition
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import collections

class Controller(Node):

    # Controller parameters
    CONTROL_GAIN = 2.5 # time constant 1/s
    SOFTENING_GAIN = 1.0 # m/s
    YAW_RATE_GAIN = 0.0
    STEERING_DAMP_GAIN = 0.0

    MAX_SAFE_DECELERATION = 8 # m/s^2
    MAX_COMFORTABLE_DECELERATION = 1 # m/s^2
    MAX_COMFORTABLE_ACCELERATION = 1 
    MIN_TIME_LOOKAHEAD = 10

    MAX_STEERING_ANGLE = 0.58294 # radians
    WHEELBASE = 0.0

    cached_odometry: Odometry = Odometry()
    cached_path: Trajectory = Trajectory()

    def __init__(self):
        super().__init__('controller_node')

        # Path
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.paths_cb,
            10
        )



    def paths_cb(self, msg: Trajectory):
        self.cached_path = msg

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg
        


        


def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()