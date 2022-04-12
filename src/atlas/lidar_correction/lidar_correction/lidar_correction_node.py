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
from typing import final
from cv2 import mean
from sensor_msgs.msg import PointCloud2, Imu
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
from shapely.prepared import prep

# For ICP
import cv2
from shapely.ops import nearest_points
from sklearn.neighbors import NearestNeighbors


class LidarCorrectionNode(Node):

    def __init__(self):
        super().__init__('lidar_correction_node')

        print("true_x,true_y,x,y,yaw,bias")

        # Create our publishers
        self.road_cloud_sub = self.create_subscription(
            PointCloud2, '/lidar/semantic/road', self.calculate_bias, 1
        )
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/zed/imu', self.cache_imu, 10
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

        self.road_grid_debug_pub = self.create_publisher(
            PointCloud2, '/atlas/debug/road_grid', 10
        )

        self.icp_result_pub = self.create_publisher(
            PointCloud2, '/atlas/debug/icp_result', 10
        )

        self.icp_input_pub = self.create_publisher(
            PointCloud2, '/atlas/debug/icp_input', 10
        )

        self.result_odom_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/atlas/corrected_pose', 10
        )

        self.true_odom_sub = self.create_subscription(
            Odometry, '/carla/odom', self.true_odom_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10)

        self.true_odom = Odometry()
        self.gnss_odom = Odometry()
        self.declare_parameter("iteration_count", 3)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.road_boundary = None
        self.map_bl_tf = TransformStamped()
        self.yaw_velocity = 0.0

        self.bias = [0.0, 0.0]
        self.idx = 1
        self.start = time.time()

    def cache_imu(self, msg: Imu):
        self.yaw_velocity = msg.angular_velocity.z

    def poly_sub_cb(self, msg: PolygonArray):
        bl_map_tf = TransformStamped()
        try:
            # Destination base_link is listed first in args
            bl_map_tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return
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

        loc = bl_map_tf.transform.translation
        nearby_circle = ShapelyPoint(loc.x, loc.y).buffer(12)
        big_ole_polygon = unary_union(polygons).intersection(nearby_circle)

        if isinstance(big_ole_polygon, MultiPolygon):
            big_ole_polygon = big_ole_polygon.geoms[0]

        if not isinstance(big_ole_polygon, ShapelyPolygon):
            self.get_logger().warn("Polygon result is not of type ShapelyPolygon. Skipping.")
            return

        viz_msg = Marker()
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.header.frame_id = 'map'  # TODO: Update this after transform
        viz_msg.type = Marker.LINE_STRIP
        viz_msg.color = ColorRGBA(
            r=0.0, g=0.5, b=1.0, a=1.0
        )
        viz_msg.frame_locked = True
        viz_msg.ns = 'nearby_poly_merged_map'

        viz_msg.scale.x = 1.0

        for pt in big_ole_polygon.exterior.coords:
            viz_msg.points.append(Point(
                x=pt[0], y=pt[1]
            ))
        viz_msgs.markers.append(viz_msg)
        self.poly_viz_pub.publish(viz_msgs)

        self.poly_viz_pub.publish(viz_msgs)
        self.road_boundary = big_ole_polygon.simplify(
            0.4, preserve_topology=False)  # Done! "Save" result

    def preprocessPoints(self, pts):
        # Filter out faraway points
        pts = pts[np.logical_and(
            np.logical_and(
                pts['x'] <= 8.0, pts['y'] < 10),
            np.logical_and(pts['y'] > -6, pts['z'] < 0.5))]
        pts = pts[::8]  # Downsample, only keeping every nth point
        # Shift the point cloud n meters closer to the car. This is a temporary fix.
        pts['x'] -= 2.0

        pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, pts)
        pcd_msg.header.frame_id = 'base_link'
        pcd_msg.header.stamp = self.get_clock().now().to_msg()

        self.icp_input_pub.publish(pcd_msg)

        return pts

    def true_odom_cb(self, msg: Odometry):
        self.true_odom = msg

    def gnss_cb(self, msg: Odometry):
        self.gnss_odom = msg

    def calculate_bias(self, msg: PointCloud2):
        start = time.time()

        # if abs(self.yaw_velocity) > 0.3:
        #     # self.get_logger().info("Turning too fast, skipping de-bias")
        #     return

        road_bound = self.road_boundary

        try:
            # Destination base_link is listed first in args
            self.map_bl_tf = self.tf_buffer.lookup_transform(
                'odom', 'map', rclpy.time.Time(seconds=0, nanoseconds=0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return

        # Is our HD map data available? If not, skip.
        if road_bound is None:
            self.get_logger().warn("Road boundary is unavailable, skipping de-bias.")
            return

        # Transform road boundary to base_link (car's reference frame)
        pts = []
        if not isinstance(road_bound, ShapelyPolygon):
            self.get_logger().warn("Multipolygons not supported, skipping de-bias.")
            return
        for map_pt in road_bound.exterior.coords:
            map_pt: ShapelyPoint

            bl_x = map_pt[0]
            bl_y = map_pt[1]
            pts.append([bl_x, bl_y, 0.0])
        map_pts = np.array(pts)
        quat = [
            self.map_bl_tf.transform.rotation.x,
            self.map_bl_tf.transform.rotation.y,
            self.map_bl_tf.transform.rotation.z,
            self.map_bl_tf.transform.rotation.w
        ]
        r = R.from_quat(quat)
        # xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        trans: Vector3 = self.map_bl_tf.transform.translation
        if len(map_pts) < 1:
            self.get_logger().warn("Map points too short, skipping.")
            return
        map_pts = r.apply(map_pts)
        map_pts[:, 0] += trans.x
        map_pts[:, 1] += trans.y

        # Visualize our road boundary
        viz_msg = Marker()
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.header.frame_id = 'base_link'  # TODO: Update this after transform
        viz_msg.type = Marker.LINE_STRIP
        viz_msg.color = ColorRGBA(
            r=0.0, g=0.5, b=1.0, a=1.0
        )
        viz_msg.frame_locked = True
        viz_msg.ns = 'nearby_poly_merged_bl'

        viz_msg.scale.x = 1.0
        if not isinstance(road_bound, ShapelyPolygon):
            self.get_logger().warn("Failed to generate ShapelyPolygon, skipping.")
            return

        for pt in map_pts:
            viz_msg.points.append(Point(
                x=pt[0], y=pt[1]
            ))
        viz_msgs = MarkerArray()
        viz_msgs.markers.append(viz_msg)
        self.poly_viz_pub.publish(viz_msgs)

        road_bound = ShapelyPolygon(map_pts)

        # Pre-process our road pointcloud
        src_original = rnp.numpify(msg)
        src_3d = self.preprocessPoints(src_original)
        if src_3d is None:
            self.get_logger().warn("Failed to filter road points, skipping.")
            return
        # These include points inside the road boundary, which aren't useful to us
        src_all = np.array([src_3d['x'], src_3d['y']]).T
        if src_all.shape[1] != 2:
            self.get_logger().warn("Filtered road points not in expected shape, skipping.")
            return

        if len(road_bound.exterior.coords) < 4:
            self.get_logger().warn("Filtered road points too short, skipping.")
            return

        outside_count = 0
        road_grid = self.get_road_grid(road_bound, 0.4)
        for pt in src_all:
            shapely_pt = ShapelyPoint(pt)
            if not shapely_pt.within(road_bound):
                # Possible slowdown...
                # if dist < 1.0:
                outside_count += 1

        if outside_count < 10:
            self.get_logger().info("Not enough rogue points, skipping.")
            return

        '''
        ICP!
        '''
        # print("Pre-ICP: {:.2f} ms".format((time.time() - start)*1000))
        res = self.icp(road_grid, src_all, max_iterations=20,
                       convergence_rotation_threshold=1.0, verbose=True)
        print("Post-ICP: {:.2f} ms".format((time.time() - start)*1000))
        if res is None or len(res) < 2:
            print("ICP failed: Where's the result?")
            return
        tf, np_pts = res
        # print(f"Y: {tf[1,2]}")
        corrected_y = tf[1, 2]

        data = np.zeros(len(np_pts), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        np_pts = np.array(
            np_pts)
        data['x'] = np_pts[:, 0]

        data['y'] = np_pts[:, 1]
        data['z'] = 2.0
        data = data[data['x'] > 0]
        msg = rnp.msgify(PointCloud2, data)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.icp_result_pub.publish(msg)

        self.visualize_tf_result(tf)

        try:
            # Destination base_link is listed first in args
            map_bl_tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))
            result_pose = PoseWithCovarianceStamped()
            result_pose.header.stamp = self.get_clock().now().to_msg()
            result_pose.header.frame_id = 'map'

            # self.lateral_corrections.append(mean_y*K*-1)
            # if len(self.lateral_corrections) > 16:
            #     self.lateral_corrections.pop(0)
            # mean_correction = sum(self.lateral_corrections) / \
            #     len(self.lateral_corrections)
            trans = map_bl_tf.transform.translation
            result_pose.pose.pose.position.x = trans.x + self.bias[0]
            result_pose.pose.pose.position.y = trans.y + self.bias[1]

            result_pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.result_odom_pub.publish(result_pose)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return

        map_bl_tf = self.tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))
        # mean_y /= outside_count
        yaw = 2*math.asin(map_bl_tf.transform.rotation.z)
        transl = map_bl_tf.transform.translation
        map_bl_tf = self.tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))

        # We now have a vector along the y axis (either the vehicle's left or right) representing our bias
        # We should rotate this using the inverse of the map->bl transform,
        # which will give us a map-level bias vector that we can use to correct GNSS data
        # Kp = 1*(outside_count**2/len(src_all))
        K = 6.0
        corr_x = K*(corrected_y*math.cos(math.pi/2+yaw))
        corr_y = K*(corrected_y*math.sin(math.pi/2+yaw))
        corr_x = min(corr_x, 1.0)
        corr_x = max(corr_x, -1.0)
        corr_y = min(corr_y, 1.0)
        corr_y = max(corr_y, -1.0)

        # self.bias[0] += corr_x*0.05
        # self.bias[1] += corr_y*0.05
        self.bias[0] -= self.bias[0]/self.idx
        self.bias[0] += corr_x / self.idx
        self.bias[1] -= self.bias[1]/self.idx
        self.bias[1] += corr_y / self.idx
        self.idx += 1

        self.bias[0] = min(self.bias[0], 2.0)
        self.bias[0] = max(self.bias[0], -2.0)
        self.bias[1] = min(self.bias[1], 2.0)
        self.bias[1] = max(self.bias[1], -2.0)

        # if abs(mean_y) < 2:
        #     return

        # TODO: Add our corrected rotation
        # result_pose.pose.pose.orientation = self.map_bl_tf.transform.rotation

        # if mean_y > 0:
        #     print("MOVE RIGHT {:.1f}".format(mean_y))
        # else:
        #     print("MOVE LEFT {:.1f}".format(
        #         mean_y))
        # print(
        #     "{:.2f},{:.2f},{:.2f}, {:.2f},{:.2f},{:.2f},{:.2f}".format(time.time()-self.start, transl.x, transl.y, corr_x, corr_y, self.bias[0], self.bias[1]))

        # print("Total: {:.2f} ms".format((time.time() - start)*1000))
        # print(corrected_y)

    def publish_correction_arrow(self, arrow_ax, arrow_ay, arrow_bx, arrow_by):
        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0
        )
        viz_msg.header.frame_id = 'map'
        viz_msg.type = Marker.ARROW
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.frame_locked = True
        viz_msg.ns = 'correction_arrow'
        viz_msg.scale.x = 0.3
        viz_msg.scale.y = 0.6
        viz_msg.points.append(Point(
            x=arrow_ax,
            y=arrow_ay
        ))
        viz_msg.points.append(Point(
            x=arrow_bx,
            y=arrow_by
        ))
        viz_msg.pose.position.z = 2.0

        self.arrow_viz_pub.publish(viz_msg)

    def visualize_tf_result(self, tf):
        # print(tf)
        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0
        )
        viz_msg.header.frame_id = 'base_link'
        viz_msg.type = Marker.ARROW
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.frame_locked = True
        viz_msg.ns = 'correction_arrow'
        viz_msg.scale.x = 0.3
        viz_msg.scale.y = 0.6
        viz_msg.points.append(Point(
            x=0.0,
            y=0.0
        ))
        viz_msg.points.append(Point(
            x=tf[0, 2]*-1,
            y=tf[1, 2]*-1
        ))

        # viz_msg.pose.position.x =
        # viz_msg.pose.position.y =
        # viz_msg.pose.position.z = 2.0
        # print(f"Yaw: {math.acos(tf[0,0])}")

        viz_msg.pose.orientation.z = math.sin(math.acos(tf[0, 0])/2)

        self.arrow_viz_pub.publish(viz_msg)

    def publish_correction_arrow_bl(self, arrow_x, arrow_y):
        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0
        )
        viz_msg.header.frame_id = 'base_link'
        viz_msg.type = Marker.ARROW
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.frame_locked = True
        viz_msg.ns = 'correction_arrow'
        viz_msg.scale.x = 0.3
        viz_msg.scale.y = 0.6
        viz_msg.points.append(Point(
            x=0.0,
            y=0.0
        ))
        viz_msg.points.append(Point(
            x=0.0,
            y=arrow_y*2
        ))
        viz_msg.pose.position.z = 2.0

        self.arrow_viz_pub.publish(viz_msg)

    # This is from Clay Flannigan's ICP implementation:
    # https://github.com/ClayFlannigan/icp/blob/master/icp.py
    # WSH.

    def point_based_matching(self, point_pairs):
        """
        This function is based on the paper "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
        by F. Lu and E. Milios.

        :param point_pairs: the matched point pairs [((x1, y1), (x1', y1')), ..., ((xi, yi), (xi', yi')), ...]
        :return: the rotation angle and the 2D translation (x, y) to be applied for matching the given pairs of points
        """

        x_mean = 0
        y_mean = 0
        xp_mean = 0
        yp_mean = 0
        n = len(point_pairs)

        if n == 0:
            return None, None, None

        for pair in point_pairs:

            (x, y), (xp, yp) = pair

            x_mean += x
            y_mean += y
            xp_mean += xp
            yp_mean += yp

        x_mean /= n
        y_mean /= n
        xp_mean /= n
        yp_mean /= n

        s_x_xp = 0
        s_y_yp = 0
        s_x_yp = 0
        s_y_xp = 0
        for pair in point_pairs:

            (x, y), (xp, yp) = pair

            s_x_xp += (x - x_mean)*(xp - xp_mean)
            s_y_yp += (y - y_mean)*(yp - yp_mean)
            s_x_yp += (x - x_mean)*(yp - yp_mean)
            s_y_xp += (y - y_mean)*(xp - xp_mean)

        rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
        translation_x = xp_mean - \
            (x_mean*math.cos(rot_angle) - y_mean*math.sin(rot_angle))
        translation_y = yp_mean - \
            (x_mean*math.sin(rot_angle) + y_mean*math.cos(rot_angle))

        return rot_angle, translation_x, translation_y

    def icp(self, reference_points, points, max_iterations=100, distance_threshold=3.0, convergence_translation_threshold=0.05,
            convergence_rotation_threshold=1e-2, point_pairs_threshold=10, verbose=False):
        """
        An implementation of the Iterative Closest Point algorithm that matches a set of M 2D points to another set
        of N 2D (reference) points.

        :param reference_points: the reference point set as a numpy array (N x 2)
        :param points: the point that should be aligned to the reference_points set as a numpy array (M x 2)
        :param max_iterations: the maximum number of iteration to be executed
        :param distance_threshold: the distance threshold between two points in order to be considered as a pair
        :param convergence_translation_threshold: the threshold for the translation parameters (x and y) for the
                                                transformation to be considered converged
        :param convergence_rotation_threshold: the threshold for the rotation angle (in rad) for the transformation
                                                to be considered converged
        :param point_pairs_threshold: the minimum number of point pairs the should exist
        :param verbose: whether to print informative messages about the process (default: False)
        :return: the transformation history as a list of numpy arrays containing the rotation (R) and translation (T)
                transformation in each iteration in the format [R | T] and the aligned points as a numpy array M x 2
        """

        transformation_history = []
        r_stack = []
        transl_x = 0.0
        transl_y = 0.0
        if reference_points is None:
            print("ICP skip: No road points were given.")
            return
        nbrs = NearestNeighbors(
            n_neighbors=1, algorithm='kd_tree').fit(reference_points)

        for iter_num in range(max_iterations):
            if verbose:
                print('------ iteration', iter_num, '------')

            closest_point_pairs = []  # list of point correspondences for closest point rule

            distances, indices = nbrs.kneighbors(points)
            for nn_index in range(len(distances)):
                if distances[nn_index][0] < distance_threshold:
                    closest_point_pairs.append(
                        (points[nn_index], reference_points[indices[nn_index][0]]))

            # if only few point pairs, stop process
            if verbose:
                print('number of pairs found:', len(closest_point_pairs))
            if len(closest_point_pairs) < point_pairs_threshold:
                if verbose:
                    print('No better solution can be found (very few point pairs)!')
                break

            # compute translation and rotation using point correspondences
            closest_rot_angle, closest_translation_x, closest_translation_y = self.point_based_matching(
                closest_point_pairs)
            if closest_rot_angle is not None:
                if verbose:
                    print('Rotation:', math.degrees(
                        closest_rot_angle), 'degrees')
                    print('Translation:', closest_translation_x,
                          closest_translation_y)
            if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
                if verbose:
                    print('No better solution can be found!')
                break

            # transform 'points' (using the calculated rotation and translation)
            c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
            rot = np.array([[c, -s],
                            [s, c]])
            rot_fixed = np.array([[c, -s],
                                  [s, c],
                                  [0, 0]])
            r_stack.append(rot)
            transl_x += closest_translation_x
            transl_y += closest_translation_y
            # aligned_points = np.dot(points, rot.T)
            # aligned_points[:, 0] += closest_translation_x
            points[:, 1] += closest_translation_y

            # update 'points' for the next iteration
            # points = aligned_points

            # update transformation history
            transformation_history.append(
                np.hstack((rot_fixed, np.array([[closest_translation_x], [closest_translation_y], [1.0]]))))

            # check convergence
            if (abs(closest_rot_angle) < convergence_rotation_threshold) \
                    and (abs(closest_translation_x) < convergence_translation_threshold) \
                    and (abs(closest_translation_y) < convergence_translation_threshold):
                if verbose:
                    print('Converged!')
                break

        if len(transformation_history) < 1:
            self.get_logger().warn("ICP skip. No transforms in history.")
            return []

        init_tf = transformation_history[0]
        final_tf = init_tf
        for tf in transformation_history[1:]:
            final_tf = np.dot(tf, final_tf)

        return final_tf, points

    def get_road_grid(self, polygon: ShapelyPolygon, res):
        valid_points = []
        latmin, lonmin, latmax, lonmax = polygon.bounds

        # create prepared polygon
        prep_polygon = prep(polygon)

        # construct a rectangular mesh
        points = []
        for lat in np.arange(latmin, latmax, res):
            for lon in np.arange(lonmin, lonmax, res):
                points.append(ShapelyPoint((round(lat, 4), round(lon, 4))))

        # validate if each point falls inside shape using
        # the prepared polygon
        valid_points.extend(filter(prep_polygon.contains, points))
        np_pts = []
        for spt in valid_points:
            np_pts.append([spt.x, spt.y])
        data = np.zeros(len(np_pts), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.float32),
            ('g', np.float32),
            ('b', np.float32)
        ])
        np_pts = np.array(
            np_pts)
        if len(np_pts) < 1:
            "Road grid skip: No valid points found."
            return
        data['x'] = np_pts[:, 0]

        data['y'] = np_pts[:, 1]
        data['z'] = 2.0
        data['r'] = 227.0
        data['g'] = 181.0
        data['b'] = 5.0
        data = data[data['x'] > 1]
        data = data[data['x'] < 12]
        msg = rnp.msgify(PointCloud2, data)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.road_grid_debug_pub.publish(msg)
        return np_pts


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
