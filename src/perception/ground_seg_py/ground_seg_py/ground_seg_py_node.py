import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import sys
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32

import image_geometry
import matplotlib.pyplot as plt

import struct


class GroundSegNode(Node):

    def __init__(self):
        super().__init__('ground_seg_py_node')

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )

        self.raw_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.point_cloud_cb, 10
        )

        self.filtered_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/filtered', 10
        )


    def clock_cb(self, msg: Clock):
        self.carla_clock = msg


    def point_cloud_cb(self, msg: PointCloud2):
        self.filtered_lidar_pud.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    ground_seg_node = GroundSegNode()

    rclpy.spin(ground_seg_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ground_seg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
