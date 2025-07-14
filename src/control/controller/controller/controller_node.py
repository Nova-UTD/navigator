#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2023 - https://github.com/Nova-UTD/navigator/
# Daniel Vayman - Daniel.Vayman@utdallas.edu

from typing import List
from navigator_msgs.msg import VehicleControl
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sin, cos, atan2
import numpy as np
from libs import normalise_angle
from tf_transformations import euler_from_quaternion


from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import collections

class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Controller parameters
        self.CONTROL_GAIN = 2.5  # time constant 1/s
        self.SOFTENING_GAIN = 1.0  # m/s
        self.YAW_RATE_GAIN = 0.0
        self.STEERING_DAMP_GAIN = 0.0

        self.MAX_SAFE_DECELERATION = 8  # m/s^2
        self.MAX_COMFORTABLE_DECELERATION = 1  # m/s^2
        self.MAX_COMFORTABLE_ACCELERATION = 1
        self.MIN_TIME_LOOKAHEAD = 10
        self.TARGET_VELOCITY = 5.0  # m/s, 11.185 mph

        self.MAX_STEERING_ANGLE = 0.58294  # radians
        self.WHEELBASE = 0.0

        self.cached_odometry: Odometry = Odometry()
        self.cached_path: Path = Path()

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

        # TODO: CREATE SUBSCRIBER FOR STEERING ANGLE

        # Vehicle Control
        self.control_pub = self.create_publisher(
            VehicleControl,
            '/vehicle/control',
            1
        )

    #TODO: PUBLISH CONTROL MESSAGE ON A TIMER

    def paths_cb(self, msg: Path):
        self.cached_path = msg
        self.stanley_control()

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg
        orientation_q = msg.pose.pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        _, _, yaw = euler_from_quaternion(q)

        self.yaw = yaw
        
    """ Finds the closest index on the path to the vehicle front axle """
    def find_target_path_id(self):
        # yaw = self.yaw # Current vehicle yaw
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
        yaw_error = normalise_angle(self.cached_path.poses.pose.orientation.z[target_index] - self.yaw)
        return yaw_error

    """ Cross track error (lateral distance from path) """
    def calculate_crosstrack_term(self, dx, dy, absolute_error):
        front_axle_vector = np.array([sin(self.yaw), -cos(self.yaw)]) # TODO: Check if negation is correct (visualize vector)
        nearest_path_vector = np.array([dx, dy])
        crosstrack_error = np.sign(nearest_path_vector@front_axle_vector) * absolute_error

        crosstrack_steering_error = atan2((self.CONTROL_GAIN * crosstrack_error), (self.SOFTENING_GAIN + self.TARGET_VELOCITY))

        return crosstrack_steering_error, crosstrack_error

    """ Damping term for yaw rate """
    def calculate_yaw_rate_term(self):
        yaw_rate_damping = self.YAW_RATE_GAIN * self.TARGET_VELOCITY * sin(self.steering_angle)
        return yaw_rate_damping

    def calculate_steering_delay_term(self, computed_steering_angle):

        steering_delay_error = self.STEERING_DAMP_GAIN * (computed_steering_angle - self.steering_angle)

        return steering_delay_error

    """ Implemented stanley controller """
    def stanley_control(self):
        target_index, dx, dy, absolute_error = self.find_target_path_id()
        yaw_error = self.calculate_yaw_term(target_index)
        crosstrack_steering_error, crosstrack_error = self.calculate_crosstrack_term(dx, dy, absolute_error)
        yaw_rate_damping = self.calculate_yaw_rate_term()

        desired_steering_angle = yaw_error + crosstrack_steering_error + yaw_rate_damping

        # Constrains steering angle to the vehicle limits
        desired_steering_angle += self.calculate_steering_delay_term(desired_steering_angle)
        limited_steering_angle = np.clip(desired_steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        return limited_steering_angle, target_index, crosstrack_error #TODO: Change to adjusting control message
    

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