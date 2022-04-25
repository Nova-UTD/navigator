#!/usr/bin/python

'''
Nova at UT Dallas, 2022

The Navigator Simulation Bridge for CARLA

The goal is to mimick Hail Bopp as much as possible.

Targetted sensors:
- GNSS (GPS)
✓ IMU ()
- Front and rear Lidar
✓ Front  RGB camera
✓ Front depth camera
- CARLA ground truths for
    - Detected objects
    ✓ Car's odometry (position, orientation, speed)
    ✓ CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

Todos:
- Specific todos are dispersed in this script. General ones are here.
- Ensure all sensors publish in ROS coordinate system, NOT Unreal Engine's.

'''

import cv2
import time
from scipy import rand
from sensor_msgs.msg import PointCloud2, Image, Imu
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, PoseWithCovarianceStamped
from voltron_msgs.msg import Obstacle3DArray, Obstacle3D, BoundingBox3D, BoundingBox2D, Obstacle2D, Obstacle2DArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry  # For GPS, ground truth
from std_msgs.msg import Bool, Header, Float32, ColorRGBA
import math
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_msgs
from tf2_ros import TransformException, TransformStamped
import pymap3d
from cv_bridge import CvBridge
import numpy as np
import ros2_numpy as rnp
from rclpy.node import Node
import rclpy
from scipy.spatial.transform import Rotation as R

# FAST_GICP
import pygicp
import open3d


# Image format conversion

'''
CHECKLIST
=========

Publish to ROS:
- [x] RGB image from left camera (15 Hz)
- [x] Depth map (15 Hz)
- [-] Point cloud -- Skipped, too slow for now
- [x] Array of detected objects
- [x] Pose data (with sensor fusion of optical odom) (max Hz)

'''


class ScanMatchingNode(Node):

    def __init__(self):
        super().__init__('scan_matcher')
        self.get_logger().info("Hello, world!")

        # Read our map
        self.get_logger().info("Reading map...")
        map_file = open3d.io.read_point_cloud(
            '/home/main/navigator-2/data/maps/grand_loop/grand_loop.pcd')
        map_cloud = np.asarray(map_file.points)

        self.pcd_source = pygicp.downsample(map_cloud, 0.25)
        self.get_logger().info(
            f"Map loaded with shape {self.pcd_source.shape}")
        print(self.pcd_source)

        self.map_pub = self.create_publisher(PointCloud2, '/map/pcd', 1)

        # Publish the map ONCE
        self.publish_cloud_from_array(self.pcd_source, 'map')

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar_fused', self.lidar_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10)

        self.initial_guess = None

    def gnss_cb(self, msg: Odometry):
        self.initial_guess = msg

    def lidar_cb(self, msg: PointCloud2):
        if self.initial_guess is None:
            self.get_logger().warning(
                "Initial guess from GNSS not yet received, skipping alignment.")

        self.get_logger().info("Trying alignment")

    def publish_cloud_from_array(self, arr, frame_id: str):
        data = np.zeros(arr.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        print(arr.shape)
        print(data.shape)
        print(arr[0])
        data['x'] = arr[:, 0]
        data['y'] = arr[:, 1]
        data['z'] = arr[:, 2]
        msg: PointCloud2 = rnp.msgify(PointCloud2, data)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    scan_matcher = ScanMatchingNode()
    rclpy.spin(scan_matcher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
