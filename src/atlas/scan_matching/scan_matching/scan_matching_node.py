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

#import cv2
import time
from scipy import rand
from sensor_msgs.msg import PointCloud2, Image, Imu
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
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
# from cv_bridge import CvBridge
import numpy as np
import ros2_numpy as rnp
from rclpy.node import Node
import rclpy
from scipy.spatial.transform import Rotation as R

# FAST_GICP
# import pygicp
import open3d as o3d


class ScanMatchingNode(Node):

    def __init__(self):
        super().__init__('scan_matcher')
        # Read our map
        self.get_logger().info("Reading map...")
        # map_file = o3d.io.read_point_cloud(
        #     '/home/wheitman/Downloads/grand_loop_fullsize.pcd')
        map_file = o3d.io.read_point_cloud(
            'data/maps/grand_loop/grand_loop.pcd')
        self.map_cloud = np.asarray(map_file.points)

        self.get_logger().info(
            f"Map loaded with shape {self.map_cloud.shape}")

        self.map_pub = self.create_publisher(PointCloud2, '/map/pcd', 1)
        self.map_pub_timer = self.create_timer(5, self.publish_map)

        # self.lidar_sub = self.create_subscription(
        #     PointCloud2, '/lidar_fused', self.lidar_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10)
        self.cached_gnss = None

        self.result_odom_pub = self.create_publisher(
            Odometry, '/atlas/odom', 10)

    def matrix_to_odom(self, result_matrix):
        result_odom = Odometry()
        result_odom.pose.pose.position.x = result_matrix[0, 3].item()
        result_odom.pose.pose.position.y = result_matrix[1, 3].item()
        result_odom.pose.pose.position.z = result_matrix[2, 3].item()

        result_quat = R.from_matrix(result_matrix[0:3, 0:3]).as_quat()
        result_odom.pose.pose.orientation.x = result_quat[0].item()
        result_odom.pose.pose.orientation.y = result_quat[1].item()
        result_odom.pose.pose.orientation.z = result_quat[2].item()
        result_odom.pose.pose.orientation.w = result_quat[3].item()

        result_odom.header.stamp = self.get_clock().now().to_msg()
        result_odom.header.frame_id = 'map'
        result_odom.child_frame_id = 'odom'
        return result_odom

    def publish_cloud_from_array(self, arr, frame_id: str, publisher):
        data = np.zeros(arr.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        data['x'] = arr[:, 0]
        data['y'] = arr[:, 1]
        data['z'] = arr[:, 2]
        msg: PointCloud2 = rnp.msgify(PointCloud2, data)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        publisher.publish(msg)

    def align(self, moving, fixed, initial_T):
        # Transform by initial tf
        moving_o3d = o3d.geometry.PointCloud()
        moving_o3d.points = o3d.utility.Vector3dVector(moving)
        moving_o3d = moving_o3d.transform(initial_T)
        moving_full = moving

        moving = pygicp.downsample(moving, 3.0)

        ndt = pygicp.FastGICP()
        ndt.set_input_target(fixed)
        ndt.set_input_source(moving)
        # gicp.set_correspondence_randomness(1000)
        ndt.set_resolution(2.0)
        # matrix = ndt.align(
        #     initial_guess=initial_T
        # )

        return matrix

    def publish_map(self):
        self.publish_cloud_from_array(self.map_cloud, 'map', self.map_pub)

    def gnss_cb(self, msg: Odometry):
        self.cached_gnss = msg

    def lidar_cb(self, msg: PointCloud2):
        # Check if our initial guess is available
        if self.cached_gnss is None:
            self.get_logger().warning(
                "Initial guess from GNSS not yet received, skipping alignment.")
            return

        self.get_logger().info("Trying alignment")

        # Prepare up our input cloud
        lidar_arr_dtype = rnp.numpify(msg)

        lidar_list = [lidar_arr_dtype['x'],
                      lidar_arr_dtype['y'], lidar_arr_dtype['z']]
        moving = np.array(lidar_list).T.reshape(-1, 3)
        moving = moving[~np.isnan(moving).any(axis=1)]

        # Filter out faraway points
        moving = moving[moving[:, 0] > -50]
        moving = moving[moving[:, 0] < 50]
        moving = moving[moving[:, 1] > -50]
        moving = moving[moving[:, 1] < 50]

        # Prepare our initial guess
        pos = self.cached_gnss.pose.pose.position
        rot = self.cached_gnss.pose.pose.orientation
        initial_trans = np.array([
            pos.x, pos.y, pos.z
        ])
        initial_quat = np.array([
            rot.x, rot.y, rot.z, rot.w
        ])
        rot_matrix = R.from_quat(initial_quat).as_matrix()
        initial_T = np.zeros((4, 4))
        initial_T[0:3, 0:3] = rot_matrix
        initial_T[0:3, 3] = initial_trans.T
        initial_T[3, 3] = 1.0

        print(initial_T)

        # try alignment >:o
        result_matrix = self.align(moving, self.map_cloud, initial_T)
        result_odom = self.matrix_to_odom(result_matrix)
        self.cached_gnss = result_odom
        self.result_odom_pub.publish(result_odom)


def main(args=None):
    rclpy.init(args=args)
    scan_matcher = ScanMatchingNode()
    rclpy.spin(scan_matcher)
    scan_matcher.destroy_node()
    gnssfile.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()