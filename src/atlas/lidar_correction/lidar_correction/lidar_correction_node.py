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

from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, TwistWithCovarianceStamped, Polygon
from voltron_msgs.msg import PeddlePosition, SteeringPosition, Obstacle3DArray, Obstacle3D, BoundingBox3D, PolygonArray
from nav_msgs.msg import Odometry  # For GPS, ground truth
from std_msgs.msg import Bool, Header, Float32
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

# Polygon intersection stuff
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


class LidarCorrectionNode(Node):

    def __init__(self):
        super().__init__('lidar_correction_node')

        # Create our publishers
        self.road_cloud_sub = self.create_subscription(
            PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
        )
        self.nearby_lane_poly_sub = self.create_subscription(
            PolygonArray, '/atlas/nearby_road_polygons', self.poly_sub_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nearby_lane_polys = []

    def poly_sub_cb(self, msg: PolygonArray):
        self.nearby_lane_polys.clear()
        for poly_msg in msg.polygons:
            poly_msg: Polygon
            pts = []
            for pt in poly_msg.points:
                pts.append(ShapelyPoint(pt.x, pt.y))
        polygon = ShapelyPolygon(pts)
        self.nearby_lane_polys.append(polygon)

    def calculate_bias(self, msg: PointCloud2):  # TODO: Transform points
        self.get_logger().info("Received {} points".format(msg.width))
        np_pts = rnp.numpify(msg)
        outside_x = 0.0
        outside_y = 0.0
        outside_qty = 0
        for pt in np_pts:
            # print(pt)
            sp = ShapelyPoint(pt['x'], pt['y'])
            isOutside = False
            for poly in self.nearby_lane_polys:
                if not sp.within(poly):
                    isOutside = True
                    break
            if isOutside:
                outside_x += sp.x
                outside_y += sp.y
                outside_qty += 1
        print("{} out of {}".format(outside_qty, msg.width))


def main(args=None):
    rclpy.init(args=args)

    lidar_correction_node = LidarCorrectionNode()

    rclpy.spin(lidar_correction_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_correction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
