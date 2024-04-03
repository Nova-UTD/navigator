#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2023 - https://github.com/Nova-UTD/navigator/
# Daniel Vayman - Daniel.Vayman@utdallas.edu

from typing import List
from navigator_msgs.msg import VehicleControl
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np
from libs import normalise_angle

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
    cached_path: Path = Path()

    def __init__(self):
        super().__init__('controller_node')

        # Path
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.paths_cb,
            10
        )

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gnss/odometry',
            self.odom_cb,
            10
        )

        # Vehicle Control
        self.control_pub = self.create_publisher(
            VehicleControl,
            '/vehicle/control',
            1
        )



    def paths_cb(self, msg: Path):
        self.cached_path = msg
        self.stanley_control()

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg
        
    """ Finds the closest index on the path to the vehicle front axle """
    def find_target_path_id(self):
        # yaw = self.cached_odometry.pose.pose.orientation.z # Current vehicle yaw
        ''' Front axle is at origin x,y '''
        x = self.cached_odometry.pose.pose.position.x # Current vehicle position
        y = self.cached_odometry.pose.pose.position.y # Current vehicle position

        px = self.cached_path.poses.pose.position.x # Path positions
        py = self.cached_path.poses.pose.position.y # Path positions

        dx = px - x # List of lateral distances
        dy = py - y # List of longitudinal distances

        d = np.hypot(dx, dy) # List of distances
        target_index = np.argmin(d) # Gets the smallest distance to path from ego

        return target_index, dx[target_index], dy[target_index], d[target_index]

    """ Finds yaw error from current yaw to target path index yaw"""
    def calculate_yaw_term(self, target_index):
        yaw_error = normalise_angle(self.cached_path.poses.pose.orientation.z[target_index] - self.cached_odometry.pose.pose.orientation.z)
        return yaw_error


    def stanley_control(self):
        target_index, dx, dy, absolute_error = self.find_target_path_id()
        yaw_error = self.calculate_yaw_term(target_index)


        


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