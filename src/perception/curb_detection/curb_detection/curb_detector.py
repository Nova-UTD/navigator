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
from os.path import exists
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2


class CurbDetector(Node):
    def __init__(self):
        super().__init__('curb_detector')
        self.TOP_RING = 6
        self.MAX_HEIGHT = 0.5  # meters

        self.LOOK_DIST = 4
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'input_points',
            self.fused_lidar_cb,
            10)
        self.curb_pub = self.create_publisher(
            PointCloud2,
            'curb_points',
            10)

    def fused_lidar_cb(self, msg: PointCloud2):
        pts = rnp.numpify(msg)
        if len(pts) == 0:
            return
        pts = self.filterPoints(pts)
        curbs = self.findAllCurbBounds(pts)
        self.curb_pub.publish(self.ptArrayToMsg(curbs))

    def filterPoints(self, pts):
        pts = pts[np.logical_not(np.isnan(pts['x']))] # Remove nan values
        pts = pts[ # Crop to height, lower rings
            np.logical_and(
                pts['ring'] <= self.TOP_RING,
                pts['z'] <= self.MAX_HEIGHT
            )
        ]
        pts = pts[ # Remove all points near vehicle
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
        pts = pts[ # Remove all points to the left of the vehicle
            pts['y'] < 0]
        return pts
    
    def ptArrayToMsg(self, pts):
        pcd2: PointCloud2 = rnp.msgify(PointCloud2, pts)
        pcd2.header.stamp = self.get_clock().now().to_msg()
        pcd2.header.frame_id = 'base_link'
        return pcd2

    def findAllCurbBounds(self, pts):
        curbs = []
        for ring in range(self.TOP_RING):
            ring_pts = pts[pts['ring'] == ring]
            curbs.append(self.slide(ring_pts))
        curbs = np.concatenate(curbs)
        curbs = curbs[curbs['y'] > np.percentile(curbs['y'], 90) - 0.1]
        return curbs

    def dist(self, a, b):
        return math.sqrt(
            (a['x'] - b['x']) ** 2 +
            (a['y'] - b['y']) ** 2 +
            (a['z'] - b['z']) ** 2)

    def mirror_around(self, a, b):
        pt = b.copy()
        pt['x'] = (pt['x'] * 2) - a['x']
        pt['y'] = (pt['y'] * 2) - a['y']
        pt['z'] = (pt['z'] * 2) - a['z']
        return pt
    
    def slide(self, pts):
        dists = []
        candidates = []
        for i in range(len(pts)):
            dists.append(self.dist(pts[i - self.LOOK_DIST], pts[i]))
        for i in range(len(pts) - self.LOOK_DIST):
            this_point = pts[i]
            previous_point = pts[i - self.LOOK_DIST]
            next_point = pts[i + self.LOOK_DIST]
            next_dist = dists[i + self.LOOK_DIST]
            previous_dist = dists[i]
            if(abs(next_dist - previous_dist) > previous_dist * 2):
                candidates.append(False)
                continue
            dist = self.dist(self.mirror_around(previous_point, this_point), next_point)
            candidates.append(bool(dist > previous_dist))
        for i in range(self.LOOK_DIST):
            candidates.append(False)
        candidates = pts[candidates]
        return candidates

def main(args=None):
    rclpy.init(args=args)
    curb_detector = CurbDetector()
    rclpy.spin(curb_detector)
    curb_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
