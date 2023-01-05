'''
Package: state_estimation
   File: mcl_node.py
 Author: Will Heitman (w at heit dot mn)

A ROS2 wrapper around MCL (see mcl.py)

Subscribes to:
- LiDAR (sensor_msgs/PointCloud2)
- Speedometer (carla_msgs/CarlaSpeedometer)
- GNSS odom (nav_msgs/Odometry)

Reads:
- .pcd map file from disk

Publishes:
- MCL result (geometry_msgs/PoseWithCovarianceStamped)
    - Minimum frequency: 2 Hz
'''

import math

import numpy as np
import rclpy
import ros2_numpy as rnp
from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from state_estimation import mcl
from tf2_ros import TransformBroadcaster

from .mcl import MCL


class MCLNode(Node):

    def __init__(self):
        super().__init__('mcl_node')

        self.filter = None
        self.gnss_pose = None
        self.old_gnss_pose = None
        self.grid: np.array = None

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/lidar_semantic_filtered', self.cloud_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/odometry/gnss_smoothed', self.gnss_cb, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/grid/map', self.map_cb, 10)

    def get_motion_delta(self, old_pose, current_pose):
        if old_pose is None:
            return np.zeros(3)

        delta = current_pose-old_pose

        # Wrap heading to [0, 2*pi]
        delta[2] %= 2*np.pi
        return delta

    def cloud_cb(self, msg: PointCloud2):
        # Update our filter
        if self.filter is None:
            return

        delta = self.get_motion_delta(self.old_gnss_pose, self.gnss_pose)

        cloud_formatted = rnp.numpify(msg)
        cloud = np.vstack((cloud_formatted['x'], cloud_formatted['y'])).T

        self.filter.step(delta, cloud)
        self.old_gnss_pose = self.gnss_pose

    def gnss_cb(self, msg: Odometry):
        pose_msg = msg.pose.pose
        yaw = 2*math.acos(pose_msg.orientation.z)
        self.gnss_pose = np.array([
            pose_msg.position.x,
            pose_msg.position.y,
            yaw
        ])

    def map_cb(self, msg: OccupancyGrid):
        if self.grid is not None:
            return  # If the map is already initialized, no need to continue
        if self.gnss_pose is None:
            return  # Wait for initial guess from GNSS

        self.grid = np.asarray(msg.data,
                               dtype=np.int8).reshape(msg.info.height, msg.info.width)

        origin = msg.info.origin.position
        self.filter = MCL(self.grid, initial_pose=self.gnss_pose,
                          map_origin=np.array([origin.x, origin.y]))

        self.get_logger().info("MCL filter created")


def main(args=None):
    rclpy.init(args=args)

    mcl_node = MCLNode()

    rclpy.spin(mcl_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mcl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
