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
from os.path import exists

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2

class CurbDetector(Node):
    def __init__(self):
        super().__init__('curb_detector')

        self.TOP_RING = 6
        self.DIST_TOLERANCE = 0.04 # meters-- 6 cm
        self.MAX_HEIGHT = 0.5 # meters
        self.MIN_ANGLE = -math.pi/2
        self.MAX_ANGLE =  math.pi/2

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'input_points',
            self.front_lidar_cb,
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

        # translation = self.lidar_to_bl_tf.transform.translation
        # pts['x'] += translation.x
        # pts['y'] += translation.y
        # pts['z'] += translation.z
        # self.get_logger().info("{}".format(translation.z))

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
                        pts['x'] >= -2.0,
                        pts['x'] <= 2.0
                    )
                )
            )
        ]

        # Rotate all points to base_link
        # TODO: Remove or rework; rotations are computationally costly!
        # quat = [
        #     self.lidar_to_bl_tf.transform.rotation.x,
        #     self.lidar_to_bl_tf.transform.rotation.y,
        #     self.lidar_to_bl_tf.transform.rotation.z,
        #     self.lidar_to_bl_tf.transform.rotation.w
        # ]
        # r = R.from_quat(quat)
        # xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        
        # pts = r.apply(xyz)

        # xyz = np.array([pts['x'],pts['y'],pts['z']])
        
        # xyz = np.transpose(xyz)
        # xyz = r.apply(xyz)
        # xyz = np.transpose(xyz)
        # # xyz[0] += translation.x
        # # xyz[1] += translation.y
        # # xyz[2] += translation.z
        # pts['x'] = xyz[0]
        # pts['y'] = xyz[1]
        # pts['z'] = xyz[2]

        # for point in pts:
        #     xyz = np.array([point['x'],point['y'],point['z']])
        #     r = R.from_euler('xyz', [-0.015, -0.03, -1.45])
        #     xyz = r.apply(xyz)
        #     point['x'] = xyz[0]
        #     point['y'] = xyz[1]
        #     point['z'] = xyz[2]

        # Remove points far to side, behind
        pts = pts[
            np.logical_and(
                 np.logical_and(
                    pts['y'] < 10.0,
                    pts['y'] > -5.0
                ),
                pts['x'] > 5.0
            )
            
        ]

        # Crop by angle
        pts = pts[
            np.logical_and(
                np.arctan2(pts['y'],pts['x']) < self.MAX_ANGLE,
                np.arctan2(pts['y'],pts['x']) > self.MIN_ANGLE
            )
        ]

        angles = np.arctan2(pts['y'],pts['x'])

        return pts

    def dist_2d(self, ptA, ptB):
        # print("{} AND {}".format(ptA, ptB))
        res =  math.sqrt(
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

    def slide(self, ring_pts, min_x=4.5, linear_dist_threshold = 0.006, min_dz = 0.06, max_dz = 0.2, window=10, debug=False):
        if ring_pts.shape[0] < 10:
            # self.get_logger().warn("Received ring with < 2 points, skipping")
            return
        
        if window < 6:
            self.get_logger().warn("Window must be at least 6.")
            return
        
        for i in range(len(ring_pts)-window):
            dz = abs(ring_pts[i+window]['z'] - ring_pts[i]['z'])
            if dz > min_dz and dz < max_dz:
                linear_dist = self.distFromPoints(
                    ring_pts[i], ring_pts[int(i+window/2)], ring_pts[i+window]
                )
                if linear_dist < linear_dist_threshold:
                    linear_dist = self.distFromPoints(
                        ring_pts[i+2], ring_pts[int(i+window/2)], ring_pts[i+window-2]
                    )
                    if linear_dist < linear_dist_threshold:
                        if debug:
                            self.get_logger().info("{} lin dist, {} dz".format(linear_dist, dz))
                        return ring_pts[i]

        # for i, pt in enumerate(ring_pts[1:]):
        #     dist = self.dist_2d(pt, prev_point)
        #     if dist > self.DIST_TOLERANCE and pt['x'] > min_x:
        #         self.get_logger().info("{} from end".format(len(ring_pts)-i))
        #         return prev_point
        #     # print(dist)
        #     prev_point = pt
        print("No curb found. last: {}".format(ring_pts[:-1]))

    def searchRingForCurbs(self, ring_pts):
        # Find index of point at theta=0

        if len(ring_pts) == 0:
            return


        angles = np.arctan2(ring_pts['y'],ring_pts['x'])
        # self.get_logger().info("Avg angle{}".format(np.mean(angles)))
        abs_angles = np.abs(angles)
        zero_idx = np.argmin(abs_angles)
        # self.get_logger().info("{} @ {}".format(ring_pts[zero_idx], zero_idx))

        # ring_pts = ring_pts[angles>0]
        # angles = angles[angles>0]
        left_pts = ring_pts[:zero_idx]
        left_curb_pt = self.slide(left_pts[::-1], linear_dist_threshold=0.007, min_dz=0.03, window=10, debug=True)
        # if pt is not None:
        #     plt.scatter(-pt['y'], pt['x'], s=20)

        right_pts = ring_pts[zero_idx:]
        right_curb_pt = self.slide(right_pts[::1])
        # if pt is not None:
        #     plt.scatter(-pt['y'], pt['x'], s=20)

        # plt.title("Left Pane") 
        # plt.xlabel("y (m)") 
        # plt.ylabel("x (m)") 
        # plt.scatter(-ring_pts['y'], ring_pts['x'], s=5)
        # if pt is not None:
        #     plt.scatter(-pt['y'], pt['x'], s=20)
        # plt.show()

        return (left_curb_pt, right_curb_pt)

    def findAllCurbBounds(self, pts):
        left_curbs = []
        right_curbs = []

        for ring in range(self.TOP_RING):
            ring_pts = pts[pts['ring']==ring]
            bounds = self.searchRingForCurbs(ring_pts)
            if bounds and bounds[0] is not None:
                left_curbs.append(bounds[0])
            if bounds and bounds[1] is not None:
                right_curbs.append(bounds[1])

        return left_curbs, right_curbs

    def formPointCloud2(self, nparray, frame_id: str):
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, nparray)
        filtered_msg.header.frame_id = frame_id
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg

    def front_lidar_cb(self, msg: PointCloud2):
        if not self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
            return
        else: 
            #self.getLidarToBlTransform('lidar_front')
            pts = rnp.numpify(msg)
            pts = self.filterPoints(pts)
            # self.publishCloud(pts, msg.header.frame_id)
            
            self.curb_candidates_pub.publish(self.formPointCloud2(pts, 'base_link'))

            left_curbs, right_curbs = self.findAllCurbBounds(pts)
            left_array = np.array(left_curbs, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint8)
            ])
            left_res_msg: PointCloud2 = rnp.msgify(PointCloud2, left_array)
            left_res_msg.header.frame_id = 'base_link'
            left_res_msg.header.stamp = self.get_clock().now().to_msg()
            self.left_curb_pts_pub.publish(left_res_msg)

            right_array = np.array(right_curbs, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint8)
            ])
            right_res_msg: PointCloud2 = rnp.msgify(PointCloud2, right_array)
            right_res_msg.header.frame_id = 'base_link'
            right_res_msg.header.stamp = self.get_clock().now().to_msg()

            # self.get_logger().info(f'{right_array.shape}')

            self.right_curb_pts_pub.publish(right_res_msg)

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