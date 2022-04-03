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
        self.arrow_viz_pub = self.create_publisher(
            Marker, '/atlas/correction_arrow', 10
        )

        self.pcd_debug_pub = self.create_publisher(
            PointCloud2, '/atlas/debug/pcd', 10
        )

        self.declare_parameter("iteration_count", 3)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.road_boundary = None
        self.bl_map_tf = TransformStamped()

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

        # Form a PolygonStamped for viz

        self.poly_viz_pub.publish(viz_msgs)
        self.road_boundary = big_ole_polygon.simplify(
            0.2, preserve_topology=False)  # Done! "Save" result

    def icp(pts, poly: ShapelyPolygon):
        num_iterations = self.get_parameter('iteration_count').value
        # for i in range(num_iterations):

        # https://stackoverflow.com/questions/20120384/iterative-closest-point-icp-implementation-on-python

    def preprocessPoints(self, pts):

        # print(pts)

        # Find our base_link->map transform
        try:
            # Destination map is listed first in args
            self.bl_map_tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return

        translation = self.bl_map_tf.transform.translation
        # pts['x'] += translation.x
        # pts['y'] += translation.y
        # pts['z'] += translation.z
        # self.get_logger().info("{}".format(translation.z))

        # Rotate all points to base_link
        # TODO: Remove or rework; rotations are computationally costly!
        quat = [
            self.bl_map_tf.transform.rotation.x,
            self.bl_map_tf.transform.rotation.y,
            self.bl_map_tf.transform.rotation.z,
            self.bl_map_tf.transform.rotation.w
        ]
        r = R.from_quat(quat)
        # xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))

        # pts = r.apply(xyz)

        xyz = np.array(
            [pts['x'].flatten(), pts['y'].flatten(), pts['z'].flatten()])
        # print(xyz)
        # print("----")

        xyz = np.transpose(xyz)
        xyz = r.apply(xyz)
        xyz = np.transpose(xyz)
        xyz[0] += translation.x
        xyz[1] += translation.y
        xyz[2] += translation.z
        pts['x'] = np.array([xyz[0]]).T
        pts['y'] = np.array([xyz[1]]).T
        pts['z'] = np.array([xyz[2]]).T

        pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, pts)
        pcd_msg.header.frame_id = 'map'
        pcd_msg.header.stamp = self.get_clock().now().to_msg()

        self.pcd_debug_pub.publish(pcd_msg)

        return pts

    def calculate_bias(self, msg: PointCloud2):
        start = time.time()

        # Is our HD map data available? If not, skip.
        if self.road_boundary is None:
            return

        src_original = rnp.numpify(msg)

        src_3d = self.preprocessPoints(src_original)
        # print(src_3d)
        # These include points inside the road boundary, which aren't useful to us
        src_all = np.array([src_3d['x'], src_3d['y']]).T[0]
        # print(src_all)
        # print("---")

        inside_count = 0
        dst_pts = []
        src_pts = []
        for pt in src_all:
            shapely_pt = ShapelyPoint(pt)
            if shapely_pt.within(self.road_boundary):
                inside_count += 1
            else:
                p1, p2 = nearest_points(self.road_boundary, shapely_pt)
                # print(p1.distance(shapely_pt))
                dst_pts.append([p1.x, p1.y])
                src_pts.append(pt)
        dst = np.array(dst_pts)  # Closest points along road boundary
        src = np.array(src_pts)  # All points outside road boundary
        # print(f"Of {len(src)}, {inside_count/len(src)*100}%")

        # print(f"TF: {Tf}")
        # print(src)
        # src = cv2.transform(src, Tf[0:2])
        # print(src)
        # if src is None:
        #     self.get_logger().warn("No points to transform. Skipping.")
        #     return

        # # Transform src from base_link->map
        # quat = [
        #     self.bl_map_tf.transform.rotation.x,
        #     self.bl_map_tf.transform.rotation.y,
        #     self.bl_map_tf.transform.rotation.z,
        #     self.bl_map_tf.transform.rotation.w
        # ]
        # r = R.from_quat(quat)
        # # src = r.apply(src)

        # outside_qty = 0
        # # for pt in src[:, 0]:

        # #     shapely_pt = ShapelyPoint(pt)

        # #     if not shapely_pt.within(self.road_boundary):
        # #         outside_qty += 1
        # print(src[:, 0])
        # if len(src[:, 0]) > 0:
        #     print(f"Of {len(src[:, 0])}, {outside_qty/len(src[:, 0])*100}%")

        # num_iterations = self.get_parameter('iteration_count').value
        # for i in range(num_iterations):
        #     # Find all points in cloud that lie outside road polygon
        #     outside_pts = np.array([])
        #     # outside_count = 0
        #     # for pt in src[0]:
        #     #     if ShapelyPoint(pt).within(self.road_boundary):
        #     #         print("Inside!")
        #     #     else:
        #     #         outside_count += 1
        #     #         # print(f"Outside: ({pt[0]},{pt[1]})")
        #     # print(f"{(len(src[0])-outside_count)/len(src[0])}% aligned")

        # # outside_qty = 0
        # # outside_x = 0.0
        # # outside_y = 0.0

        # # for pt in src:
        # #     sp = ShapelyPoint(pt['x'], pt['y'])
        # #     if not sp.within(road_poly):
        # #         outside_qty += 1
        # #         outside_x += sp.x
        # #         outside_y += sp.y
        # # if outside_qty <= 0:
        # #     return
        # # outside_x /= outside_qty
        # # outside_y /= outside_qty
        # # # print("{} of {}".format(outside_qty, msg.width))
        # # if outside_y < 0:
        # #     print("LEFT {}".format(abs(outside_y)))
        # # else:
        # #     print("RIGHT {}".format(abs(outside_y)))
        # # self.publish_correction_arrow(-1*outside_x, -1*outside_y)

        # # Publish DEBUG road boundary
        # viz_msg = Marker()
        # viz_msg.color = ColorRGBA(
        #     r=0.0, g=1.0, b=0.2, a=1.0
        # )
        # viz_msg.header.frame_id = 'map'
        # viz_msg.type = Marker.LINE_STRIP
        # viz_msg.id = 45
        # viz_msg.header.stamp = self.get_clock().now().to_msg()
        # viz_msg.scale.x = 0.3
        # viz_msg.frame_locked = True
        # viz_msg.ns = 'nearby_poly_merged_tf'
        # viz_msg.scale.x = 1.0

        # for pt in self.road_boundary.exterior.coords:
        #     viz_msg.points.append(Point(
        #         x=pt[0], y=pt[1]
        #     ))
        # viz_msgs = MarkerArray()
        # viz_msgs.markers.append(viz_msg)
        # self.poly_viz_pub.publish(viz_msgs)

        # # Publish DEBUG PCD
        # z = np.zeros((src.shape[0], 2))
        # # print(src.shape)
        # # print(z.shape)
        # src = np.hstack((src, z))
        # print(src)
        # src.dtype = [('x', np.float32),
        #              ('y', np.float32),
        #              ('z', np.float32),
        #              ('i', np.float32)
        #              ]
        # pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, src)
        # pcd_msg.header.frame_id = 'base_link'
        # pcd_msg.header.stamp = self.get_clock().now().to_msg()

        # end = time.time()
        # self.pcd_debug_pub.publish(pcd_msg)
        # # print(f"{(end - start)*1000} ms")

    def publish_correction_arrow(self, arrow_x, arrow_y):
        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0
        )
        viz_msg.header.frame_id = 'base_link'
        viz_msg.type = Marker.ARROW
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.scale.x = 0.3
        viz_msg.frame_locked = True
        viz_msg.ns = 'correction_arrow'
        viz_msg.scale.x = 0.3
        viz_msg.scale.y = 0.6
        viz_msg.points.append(Point(
            x=0.0,
            y=0.0
        ))
        viz_msg.points.append(Point(
            x=arrow_x,
            y=arrow_y
        ))

        self.arrow_viz_pub.publish(viz_msg)


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
