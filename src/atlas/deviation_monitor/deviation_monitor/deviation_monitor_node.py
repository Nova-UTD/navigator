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

from turtle import Shape
from scipy import rand
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, Polygon, PolygonStamped, Point32, PoseWithCovarianceStamped
from voltron_msgs.msg import PeddlePosition, SteeringPosition, Obstacle3DArray, Obstacle3D, BoundingBox3D, PolygonArray
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

# For process timing
import time

# Polygon intersection stuff
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon
from shapely.ops import unary_union, transform
from shapely import affinity

# For ICP
import cv2
from shapely.ops import nearest_points
from sklearn.neighbors import NearestNeighbors


class DeviationMonitorNode(Node):

    def __init__(self):
        super().__init__('deviation_monitor_node')

        # Subscribe to ground truth odom, estimated odom
        self.true_odom_sub = self.create_subscription(
            Odometry, '/carla/odom', self.true_odom_cb, 10)

        self.ukf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ukf_odom_cb, 10)

        self.deviation_pub = self.create_publisher(
            Float32, '/atlas/deviation', 10
        )

        self.declare_parameter("iteration_count", 3)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.road_boundary = None
        self.bl_map_tf = TransformStamped()
        self.true_odom = Odometry()
        self.errors = []
        self.error_history = 500

    def true_odom_cb(self, msg: Odometry):
        # print("True OD received")
        self.true_odom = msg

    def ukf_odom_cb(self, msg: Odometry):
        # print("UKF OD received")
        ukf_pos = msg.pose.pose.position
        tru_pos = self.true_odom.pose.pose.position
        transl_error = math.sqrt(
            (ukf_pos.x - tru_pos.x)**2 +
            (ukf_pos.y - tru_pos.y)**2 +
            (ukf_pos.z - tru_pos.z)**2
        )

        if len(self.errors) < self.error_history:
            self.errors.append(transl_error)
        mean_error = np.array(self.errors).mean()
        self.deviation_pub.publish(Float32(data=mean_error))
        print("Er: {}".format(mean_error))


def main(args=None):
    rclpy.init(args=args)

    deviation_monitor_node = DeviationMonitorNode()

    rclpy.spin(deviation_monitor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    deviation_monitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
