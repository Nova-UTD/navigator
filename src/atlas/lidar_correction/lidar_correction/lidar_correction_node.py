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
            PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
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

        self.pcd_debug_pub = self.create_publisher(
            PointCloud2, '/atlas/debug/pcd', 10
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

    def cache_imu(self, msg: Imu):
        self.yaw_velocity = msg.angular_velocity.z

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

        big_ole_polygon = unary_union(polygons)

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

        try:
            # Destination base_link is listed first in args
            self.map_bl_tf = self.tf_buffer.lookup_transform(
                'base_link', 'map', rclpy.time.Time(seconds=0, nanoseconds=0))

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return

    def preprocessPoints(self, pts):
        # Filter out faraway points
        pts = pts[np.logical_and(
            np.logical_and(
                pts['x'] <= 10.0, pts['y'] < 10),
            np.logical_and(pts['y'] > -6, pts['z'] < 0.5))]
        pts = pts[::1]  # Downsample, only keeping every nth point
        # Shift the point cloud n meters closer to the car. This is a temporary fix.
        pts['x'] -= 2.0

        pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, pts)
        pcd_msg.header.frame_id = 'base_link'
        pcd_msg.header.stamp = self.get_clock().now().to_msg()

        self.pcd_debug_pub.publish(pcd_msg)

        return pts

    def true_odom_cb(self, msg: Odometry):
        # print("True OD received")
        self.true_odom = msg

    def gnss_cb(self, msg: Odometry):
        # print("True OD received")
        self.gnss_odom = msg

    def calculate_bias(self, msg: PointCloud2):
        start = time.time()
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
            result_pose.pose.pose.position.x = trans.x + self.bias[0]*3.0
            result_pose.pose.pose.position.y = trans.y + self.bias[1]*3.0

            result_pose.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.result_odom_pub.publish(result_pose)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return

        if abs(self.yaw_velocity) > 0.3:
            self.get_logger().info("Turning too fast, skipping de-bias")
            return

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
        # print(src_all)
        if src_all.shape[1] != 2:
            self.get_logger().warn("Filtered road points not in expected shape, skipping.")
            return
        # print(src_all)
        # print("---")

        if len(road_bound.exterior.coords) < 4:
            self.get_logger().warn("Filtered road points too short, skipping.")
            return

        outside_count = 0
        mean_y = 0
        for pt in src_all:
            shapely_pt = ShapelyPoint(pt)
            if not shapely_pt.within(road_bound):
                # Possible slowdown...
                p1, p2 = nearest_points(road_bound, shapely_pt)
                dist = p1.distance(shapely_pt)
                # if dist < 1.0:
                outside_count += 1
                mean_y += shapely_pt.y-p1.y
                # print(pt[1])
        if outside_count < 10:
            # self.get_logger().info("No bias found, skipping.")
            return

        # T, distances, i = self.icp(src, dst)
        map_bl_tf = self.tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(seconds=0, nanoseconds=0))
        mean_y /= outside_count
        yaw = 2*math.asin(map_bl_tf.transform.rotation.z)
        transl = map_bl_tf.transform.translation
        true = self.true_odom.pose.pose.position
        # print(f"{true.x},{true.y},{transl.x},{transl.y},{yaw},{mean_y}")

        # We now have a vector along the y axis (either the vehicle's left or right) representing our bias
        # We should rotate this using the inverse of the map->bl transform,
        # which will give us a map-level bias vector that we can use to correct GNSS data
        # Kp = 1*(outside_count**2/len(src_all))
        K = 5.0 * (outside_count/len(src_all))

        corr_x = -1*(mean_y*math.cos(math.pi/2+yaw)*K)
        corr_y = -1*(mean_y*math.sin(math.pi/2+yaw)*K)
        # self.bias[0] += corr_x*0.05
        # self.bias[1] += corr_y*0.05
        self.bias[0] -= self.bias[0]/self.idx
        self.bias[0] += corr_x / self.idx
        self.bias[1] -= self.bias[1]/self.idx
        self.bias[1] += corr_y / self.idx
        self.idx += 1

        self.bias[0] = min(self.bias[0], 1.0)
        self.bias[0] = max(self.bias[0], -1.0)
        self.bias[1] = min(self.bias[1], 1.0)
        self.bias[1] = max(self.bias[1], -1.0)

        self.publish_correction_arrow(
            transl.x, transl.y, transl.x+self.bias[0]*4, transl.y+self.bias[1]*4)

        # if abs(mean_y) < 2:
        #     return

        # TODO: Add our corrected rotation
        # result_pose.pose.pose.orientation = self.map_bl_tf.transform.rotation

        # if mean_y > 0:
        #     print("MOVE RIGHT {:.1f}".format(mean_y))
        # else:
        #     print("MOVE LEFT {:.1f}".format(
        #         mean_y))
        print(f"Bias: {(self.bias[0]**2+self.bias[1]**2)**0.5}")

        end = time.time()
        # print("{:.2f} ms".format((end - start)*1000))

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

    # This is from Clay Flannigan's ICP implementation:
    # https://github.com/ClayFlannigan/icp/blob/master/icp.py
    # WSH.

    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
        A: Nxm numpy array of corresponding points
        B: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
        R: mxm rotation matrix
        t: mx1 translation vector
        '''

        assert A.shape == B.shape

        # get number of dimensions
        m = A.shape[1]

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[m-1, :] *= -1
            R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R, centroid_A.T)

        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t

        return T, R, t

    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: dst indices of the nearest neighbor
        '''

        assert src.shape == dst.shape

        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()

    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001):
        '''
        The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
        Input:
            A: Nxm numpy array of source mD points
            B: Nxm numpy array of destination mD point
            init_pose: (m+1)x(m+1) homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation that maps A on to B
            distances: Euclidean distances (errors) of the nearest neighbor
            i: number of iterations to converge
        '''

        assert A.shape == B.shape

        # get number of dimensions
        m = A.shape[1]

        # make points homogeneous, copy them to maintain the originals
        src = np.ones((m+1, A.shape[0]))
        dst = np.ones((m+1, B.shape[0]))
        src[:m, :] = np.copy(A.T)
        dst[:m, :] = np.copy(B.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(
                src[:m, :].T, dst[:m, :].T)

            # compute the transformation between the current source and nearest destination points
            T, _, _ = self.best_fit_transform(src[:m, :].T, dst[:m, indices].T)

            # update the current source
            src = np.dot(T, src)

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculate final transformation
        T, _, _ = self.best_fit_transform(A, src[:m, :].T)

        return T, distances, i


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
