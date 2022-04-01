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

from scipy import rand
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, Polygon, PolygonStamped, Point32
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

# Polygon intersection stuff
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon
from shapely.ops import unary_union, transform
from shapely import affinity


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
        self.poly_viz_pub = self.create_publisher(
            MarkerArray, '/atlas/poly_viz', 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.road_boundary = None
        self.map_bl_tf = TransformStamped()

    def poly_sub_cb(self, msg: PolygonArray):
        polygons = []
        viz_msgs = MarkerArray()
        for i, poly_msg in enumerate(msg.polygons):
            poly_msg: Polygon
            pts = []
            for pt in poly_msg.points:
                pts.append(ShapelyPoint(pt.x, pt.y))
            spoly = ShapelyPolygon(pts).buffer(0.1)
            polygons.append(spoly)
            viz_msg = Marker()
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            viz_msg.header.frame_id = 'map'  # TODO: Update this after transform
            viz_msg.type = Marker.LINE_STRIP
            viz_msg.color = ColorRGBA(
                r=0.0, g=0.5, b=1.0, a=1.0
            )
            viz_msg.frame_locked = True
            viz_msg.ns = 'nearby_lane_polygons'
            viz_msg.id = i
            viz_msg.scale.x = 0.1

            for pt in spoly.exterior.coords:
                viz_msg.points.append(Point(
                    x=pt[0], y=pt[1]
                ))
            # print(pt)

            viz_msgs.markers.append(viz_msg)

        # self.nearby_lane_polys.append(polygon)
        total_area = 0.0
        for poly in polygons:
            total_area += poly.area
        big_ole_polygon = unary_union(polygons)
        if isinstance(big_ole_polygon, MultiPolygon):
            big_ole_polygon = big_ole_polygon.geoms[0]
        # print("{} vs {}".format(total_area, big_ole_polygon.area))

        viz_msg = Marker()
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.header.frame_id = 'map'  # TODO: Update this after transform
        viz_msg.type = Marker.LINE_STRIP
        viz_msg.color = ColorRGBA(
            r=0.0, g=0.5, b=1.0, a=1.0
        )
        viz_msg.frame_locked = True
        viz_msg.ns = 'nearby_poly_merged'
        # viz_msg.id =
        viz_msg.scale.x = 0.3

        for pt in big_ole_polygon.exterior.coords:
            viz_msg.points.append(Point(
                x=pt[0], y=pt[1]
            ))
        viz_msgs.markers.append(viz_msg)

        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=0.0, g=1.0, b=0.2, a=1.0
        )
        viz_msg.header.frame_id = 'base_link'
        viz_msg.type = Marker.LINE_STRIP
        viz_msg.id = 45
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.scale.x = 0.3
        viz_msg.frame_locked = True
        viz_msg.ns = 'nearby_poly_merged_tf'
        viz_msg.scale.x = 0.3

        yaw = 2*math.asin(self.map_bl_tf.transform.rotation.z)
        x_off = self.map_bl_tf.transform.translation.x
        y_off = self.map_bl_tf.transform.translation.y
        road_bound_tfed = affinity.affine_transform(big_ole_polygon, [
            math.cos(yaw), -1 *
            math.sin(yaw), math.sin(yaw), math.cos(yaw), x_off, y_off
        ])

        for pt in road_bound_tfed.exterior.coords:
            viz_msg.points.append(Point(
                x=pt[0], y=pt[1]
            ))
        viz_msgs.markers.append(viz_msg)

        # Form a PolygonStamped for viz

        # print(big_ole_polygon.boundary)
        self.poly_viz_pub.publish(viz_msgs)
        self.road_boundary = big_ole_polygon.simplify(
            0.2, preserve_topology=False)  # Done! "Save" result

    def calculate_bias(self, msg: PointCloud2):  # TODO: Transform points
        if self.road_boundary is None:
            return
        try:
            self.map_bl_tf = self.tf_buffer.lookup_transform(
                'base_link', 'map', rclpy.time.Time(seconds=0, nanoseconds=0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')

        tfed_pts = []

        transl = self.map_bl_tf.transform.translation
        yaw = 2*math.acos(self.map_bl_tf.transform.rotation.z)
        print(yaw*180/math.pi)

        # for point in self.road_boundary.exterior.coords:
        #     tfed_pts.append(ShapelyPoint(
        #         [
        #             point[0] + transl.x,
        #             point[1] + transl.y
        #         ]
        #     ))
        yaw = 2*math.asin(self.map_bl_tf.transform.rotation.z)
        x_off = self.map_bl_tf.transform.translation.x
        y_off = self.map_bl_tf.transform.translation.y
        road_bound_tfed = affinity.affine_transform(self.road_boundary, [
            math.cos(yaw), -1 *
            math.sin(yaw), math.sin(yaw), math.cos(yaw), x_off, y_off
        ])

        # self.get_logger().info("Received {} points".format(msg.width))
        np_pts = rnp.numpify(msg)

        inside_qty = 0
        outside_qty = 0
        outside_x = 0.0
        outside_y = 0.0

        for pt in np_pts:
            sp = ShapelyPoint(pt['x'], pt['y'])
            if not sp.within(road_bound_tfed):
                outside_qty += 1
                outside_x += sp.x
                outside_y += sp.y
        if outside_qty > 0:
            print("{} of {}".format(outside_qty, msg.width))


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
