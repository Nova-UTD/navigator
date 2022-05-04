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
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'input_points',
            self.fused_lidar_cb,
            10)
        self.curb_pub = self.create_publisher(
            PointCloud2,
            'curb_points',
            10)
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

    def fused_lidar_cb(self, msg: PointCloud2):
        if not self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
            return
        else:
            pts = rnp.numpify(msg)
            pts = self.filterPoints(pts)
            curbs = self.findAllCurbBounds(pts)
        if len(curbs) == 0:
            return
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
                        pts['x'] >= -2.0,
                        pts['x'] <= 2.0
                    )
                )
            )
        ]
        pts = pts[ # Remove all points to the left of the vehicle by more than 2 meters
            pts['y'] > 2.0]
        return pts
    
    def ptArrayToMsg(self, pts):
        np_pts = np.array(pts)
        pcd2: PointCloud2 = rnp.msgify(PointCloud2, np_pts)
        pcd2.header.stamp = self.get_clock().now().to_msg()
        pcd2.header.frame_id = 'base_link'
        return pcd2

    def slide(self, ring_pts, curbs):
        last_pt = ring_pts[0]
        for pt in ring_pts[1:]:
            y_dist = abs(last_pt['y'] - pt['y'])
            z_dist = abs(last_pt['z'] - pt['z'])
            slope = z_dist / y_dist
            last_pt = pt
            if slope > 10.0:
                curbs.append(pt)

    def findAllCurbBounds(self, pts):
        curbs = []
        for ring in range(self.TOP_RING):
            ring_pts = pts[pts['ring'] == ring]
            self.slide(ring_pts, curbs)
        return curbs

def main(args=None):
    rclpy.init(args=args)
    curb_detector = CurbDetector()
    rclpy.spin(curb_detector)
    curb_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
