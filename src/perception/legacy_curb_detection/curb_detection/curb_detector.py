import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from os.path import exists

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2


class CurbDetector(Node):
    def __init__(self):
        super().__init__('curb_detector')

        self.TOP_RING = 6
        self.DIST_TOLERANCE = 0.04  # meters-- 6 cm
        self.MAX_HEIGHT = 0.5  # meters
        self.MIN_ANGLE = -math.pi/2
        self.MAX_ANGLE = math.pi/2

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'input_points',
            self.fused_lidar_cb,
            10
        )

        self.curb_candidates_pub = self.create_publisher(
            PointCloud2,
            'curb_candidates',
            10
        )

        self.left_curb_pts_pub = self.create_publisher(
            PointCloud2,
            'curb_points/left',
            10
        )

        self.right_curb_pts_pub = self.create_publisher(
            PointCloud2,
            'curb_points/right',
            10
        )

        self.dist_pub = self.create_publisher(
            Float32,
            'curb_distance',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_to_bl_tf = TransformStamped()

    def getLidarToBlTransform(self, lidar_frame: str):
        try:
            self.lidar_to_bl_tf = self.tf_buffer.lookup_transform(
                'base_link',
                lidar_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {dest_frame} to {lidar_frame}: {ex}')
            return

    def filterPoints(self, pts):
        # Remove nan values
        pts = pts[np.logical_not(np.isnan(pts['x']))]

        # Crop to height, lower rings
        pts = pts[
            np.logical_and(
                pts['ring'] <= self.TOP_RING,
                pts['z'] <= self.MAX_HEIGHT
            )
        ]

        # Remove all points near vehicle
        pts = pts[
            np.logical_not(
                np.logical_and(
                    np.logical_and(
                        pts['y'] >= -2.0,
                        pts['y'] <= 2.0
                    ),
                    np.logical_and(
                        pts['x'] >= -5.0,
                        pts['x'] <= 2.0
                    )
                )
            )
        ]

        angles = np.arctan2(pts['y'], pts['x'])

        return pts

    def dist_2d(self, ptA, ptB):
        # print("{} AND {}".format(ptA, ptB))
        res = math.sqrt(
            (ptA['x'] - ptB['x']) ** 2 +
            (ptA['y'] - ptB['y']) ** 2
        )
        return res

    def distFromPoints(self, ptStart, ptBetween, ptEnd):
        x0 = ptBetween['x']
        x1 = ptStart['x']
        x2 = ptEnd['x']
        y0 = ptBetween['y']
        y1 = ptStart['y']
        y2 = ptEnd['y']

        numerator = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))
        denominator = math.sqrt((x2-x1)**2 + (y2-y1)**2)

        return numerator/denominator

    def slide(self, ring_pts, candidate_points):
        last_pt = ring_pts[0]
        for pt in ring_pts[1:]:
            y_dist = abs(last_pt['y'] - pt['y'])
            z_dist = abs(last_pt['z'] - pt['z'])
            slope = z_dist / y_dist
            last_pt = pt
            if slope > 10.0:
                candidate_points.append(pt)

    def ptArrayToMsg(self, pts):
        np_pts = np.array(pts)
        pcd2: PointCloud2 = rnp.msgify(PointCloud2, np_pts)
        pcd2.header.stamp = self.get_clock().now().to_msg()
        pcd2.header.frame_id = 'base_link'
        return pcd2

    def k_means_cluster(self, candidate_points):
        left_mean = [0, 10, 0]
        right_mean = [0, -10, 0]

        for i in range(3):
            new_left = [0, 0, 0]
            new_right = [0, 0, 0]
            n_left_pts = 0
            n_right_pts = 0

            for pt in candidate_points:
                x_dist_to_left = (pt['x'] - left_mean[0]) ** 2
                x_dist_to_right = (pt['x'] - right_mean[0]) ** 2
                y_dist_to_left = (pt['y'] - left_mean[1]) ** 2
                y_dist_to_right = (pt['y'] - right_mean[1]) ** 2
                z_dist_to_left = (pt['z'] - left_mean[2]) ** 2
                z_dist_to_right = (pt['z'] - right_mean[2]) ** 2
                total_dist_to_left = x_dist_to_left + y_dist_to_left + z_dist_to_left
                total_dist_to_right = x_dist_to_right + y_dist_to_right + z_dist_to_right
                if(total_dist_to_left > total_dist_to_right):
                    n_left_pts += 1
                    new_left[0] += pt['x']
                    new_left[1] += pt['y']
                    new_left[2] += pt['z']
                else:
                    n_right_pts += 1
                    new_right[0] += pt['x']
                    new_right[1] += pt['y']
                    new_right[2] += pt['z']

            new_left[0] /= n_left_pts
            new_left[1] /= n_left_pts
            new_left[2] /= n_left_pts
            new_right[0] /= n_right_pts
            new_right[1] /= n_right_pts
            new_right[2] /= n_right_pts

            left_mean = new_left
            right_mean = new_right

        left_pts = []
        right_pts = []

        for pt in candidate_points:
            x_dist_to_left = (pt['x'] - left_mean[0]) ** 2
            x_dist_to_right = (pt['x'] - right_mean[0]) ** 2
            y_dist_to_left = (pt['y'] - left_mean[1]) ** 2
            y_dist_to_right = (pt['y'] - right_mean[1]) ** 2
            z_dist_to_left = (pt['z'] - left_mean[2]) ** 2
            z_dist_to_right = (pt['z'] - right_mean[2]) ** 2
            total_dist_to_left = x_dist_to_left + y_dist_to_left + z_dist_to_left
            total_dist_to_right = x_dist_to_right + y_dist_to_right + z_dist_to_right
            if(total_dist_to_left > total_dist_to_right):
                left_pts.append(pt)
            else:
                right_pts.append(pt)

        return left_pts, right_pts

    def get_mean(self, pts):
        mean = [0, 0, 0]
        for pt in pts:
            mean[0] += pt['x']
            mean[1] += pt['y']
            mean[2] += pt['z']
        n_pts = len(pts)
        mean[0] /= n_pts
        mean[1] /= n_pts
        mean[2] /= n_pts
        return mean

    def findAllCurbBounds(self, pts):
        left_curbs = []
        right_curbs = []

        candidate_points = []
        for ring in range(self.TOP_RING):
            ring_pts = pts[pts['ring'] == ring]
            self.slide(ring_pts, candidate_points)

        left_pts, right_pts = self.k_means_cluster(candidate_points)
        left_mean = self.get_mean(left_pts)
        right_mean = self.get_mean(right_pts)

        mean_x = (left_mean[0] + right_mean[0]) / 2
        mean_y = (left_mean[1] + right_mean[1]) / 2
        # delta_x = left_mean[0] - right_mean[0]
        # delta_y = left_mean[1] - right_mean[1]
        # slope = delta_y / delta_x  # Slope of a line in between these points
        # x_intercept = intercept_x - (slope * intercept_y)

        right_pts_trimmed = self.trim(right_pts, mean_x, mean_y)

        return right_pts_trimmed

    def trim(self, pts, x, y):
        pts_cut = []
        for pt in pts:
            if pt['y'] < 0:
                pts_cut.append(pt)
        pts = pts_cut

        distances = []
        for pt in pts:
            # x_d = (pt['x'] - x) ** 2
            # y_d = (pt['y'] - y) ** 2
            # distances.append(math.sqrt(x_d + y_d))
            distances.append(pt['y'])
        distances.sort()
        max_dist = distances[math.floor(len(distances) * 8 / 10)]

        points = []
        for pt in pts:
            # x_d = (pt['x'] - x) ** 2
            # y_d = (pt['y'] - y) ** 2
            # d = math.sqrt(x_d + y_d)
            d = pt['y']
            if(d > max_dist):
                points.append(pt)
        return points

    def formPointCloud2(self, nparray, frame_id: str):
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, nparray)
        filtered_msg.header.frame_id = frame_id
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg

    def fused_lidar_cb(self, msg: PointCloud2):
        if not self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
            return
        else:
            pts = rnp.numpify(msg)
            pts = self.filterPoints(pts)
            right_curbs = self.findAllCurbBounds(pts)

        if len(right_curbs) == 0:
            return

        mean_curb = self.get_mean(right_curbs)

        dist_msg = Float32()
        dist_msg.data = mean_curb[1]
        self.dist_pub.publish(dist_msg)
        self.right_curb_pts_pub.publish(self.ptArrayToMsg(right_curbs))
        # self.left_curb_pts_pub.publish(self.ptArrayToMsg(left_curbs))


def main(args=None):
    rclpy.init(args=args)

    curb_detector = CurbDetector()

    rclpy.spin(curb_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    curb_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
