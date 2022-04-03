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

        self.result_odom_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/atlas/corrected_pose', 10
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
        # Filter out faraway points
        pts = pts[pts['x'] <= 20.0]

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
        pts['x'] = np.array(xyz[0]).T
        pts['y'] = np.array(xyz[1]).T
        pts['z'] = np.array(xyz[2]).T

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
        if src_3d is None:
            return
        # print(src_3d)
        # These include points inside the road boundary, which aren't useful to us
        src_all = np.array([src_3d['x'], src_3d['y']]).T
        if src_all.shape[1] != 2:
            return
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
                if p1.distance(shapely_pt) < 2.0:
                    dst_pts.append([p1.x, p1.y])
                    src_pts.append(pt)
        dst = np.array(dst_pts)  # Closest points along road boundary
        src = np.array(src_pts)  # All points outside road boundary
        if len(src) > 0:

            if inside_count/len(src_all) > 0.95:
                print("All clear.")
                return

        if src.shape[0] < 1:
            return

        T, distances, i = self.icp(src, dst)
        x_off = T[0, 2]
        y_off = T[1, 2]
        print("Move {:.2f} right, {:.2f} forward".format(y_off, x_off))
        print("Of {}, {:.1f}%".format(
            len(src_all), inside_count/len(src_all)*100))
        self.publish_correction_arrow(T[0, 2], T[1, 2])

        result_pose = PoseWithCovarianceStamped()
        result_pose.header.stamp = self.get_clock().now().to_msg()
        result_pose.header.frame_id = 'map'
        # TODO: Add our corrected rotation
        result_pose.pose.pose.orientation = self.bl_map_tf.transform.rotation
        og_transl = self.bl_map_tf.transform.translation
        result_pose.pose.pose.position.x = og_transl.x + x_off
        result_pose.pose.pose.position.y = og_transl.y + x_off
        result_pose.pose.pose.position.z = og_transl.z
        self.result_odom_pub.publish(result_pose)

        end = time.time()
        # print("{:.2f} ms".format((end - start)*1000))

    def publish_correction_arrow(self, arrow_x, arrow_y):
        viz_msg = Marker()
        viz_msg.color = ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0
        )
        viz_msg.header.frame_id = 'map'
        viz_msg.type = Marker.ARROW
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.scale.x = 0.3
        viz_msg.frame_locked = True
        viz_msg.ns = 'correction_arrow'
        viz_msg.scale.x = 0.3
        viz_msg.scale.y = 0.6
        viz_msg.pose.position.x = self.bl_map_tf.transform.translation.x
        viz_msg.pose.position.y = self.bl_map_tf.transform.translation.y
        viz_msg.points.append(Point(
            x=0.0,
            y=0.0
        ))
        viz_msg.points.append(Point(
            x=arrow_x,
            y=arrow_y
        ))

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
