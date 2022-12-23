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
import rclpy
from state_estimation import mcl
import ros2_numpy as rnp
import numpy as np
import open3d as o3d
from rclpy.node import Node

from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster


class MCLNode(Node):

    def __init__(self):
        super().__init__('mcl_node')
        map_points = np.asarray(o3d.io.read_point_cloud(
            "/workspace/simulator/HDMaps/Town01.pcd").points)
        self.get_logger().info(str(map_points))


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
