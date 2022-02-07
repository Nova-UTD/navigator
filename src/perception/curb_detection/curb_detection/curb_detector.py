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

        self.TOP_RING = 5
        self.DIST_TOLERANCE = 0.01 # meters-- 6 cm
        self.MAX_HEIGHT = 0.5 # meters

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_front/velodyne_points',
            self.front_lidar_cb,
            10
        )

        self.curb_candidates_pub = self.create_publisher(
            PointCloud2,
            '/lidar_front/curb_candidates',
            10
        )

        self.curb_pts_pub = self.create_publisher(
            PointCloud2,
            '/lidar_front/curb_points',
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

    def filterPoints(self, pts, min_angle = math.pi/2, max_angle = -math.pi/2):
        # Remove nan values
        pts = pts[np.logical_not(np.isnan(pts['x']))]

        translation = self.lidar_to_bl_tf.transform.translation
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
        quat = [
            self.lidar_to_bl_tf.transform.rotation.x,
            self.lidar_to_bl_tf.transform.rotation.y,
            self.lidar_to_bl_tf.transform.rotation.z,
            self.lidar_to_bl_tf.transform.rotation.w
        ]
        r = R.from_quat(quat)
        # xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        
        # pts = r.apply(xyz)

        xyz = np.array([pts['x'],pts['y'],pts['z']])
        
        xyz = np.transpose(xyz)
        xyz = r.apply(xyz)
        xyz = np.transpose(xyz)
        xyz[0] += translation.x
        xyz[1] += translation.y
        xyz[2] += translation.z
        pts['x'] = xyz[0]
        pts['y'] = xyz[1]
        pts['z'] = xyz[2]
        self.get_logger().info("{}".format(xyz))

        # for point in pts:
        #     xyz = np.array([point['x'],point['y'],point['z']])
        #     r = R.from_euler('xyz', [-0.015, -0.03, -1.45])
        #     xyz = r.apply(xyz)
        #     point['x'] = xyz[0]
        #     point['y'] = xyz[1]
        #     point['z'] = xyz[2]

        # Crop by angle
        # pts = pts[
        #     np.logical_and(
        #         np.arctan2(pts['y'],pts['x']) < max_angle,
        #         np.arctan2(pts['y'],pts['x']) > min_angle
        #     )
        # ]

        finalPointCount = pts.shape[0]
        # self.get_logger().info("{}".format(pts))

        return pts

    def slideWindow(ring_pts):
        # Find index of point at theta=0
        angles = np.arctan2(ring_pts['y'],ring_pts['x'])
        abs_angles = np.abs(angles)
        zero_idx = np.argmin(abs_angles)
        # print(ring_pts[zero_idx])

        # ring_pts = ring_pts[angles>0]
        # angles = angles[angles>0]
        left_pts = ring_pts[:zero_idx]
        left_curb_pt = self.slide(left_pts[::-1])
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

    def dist_2d(self, ptA, ptB):
        # print("{} AND {}".format(ptA, ptB))
        res =  math.sqrt(
            (ptA['x'] - ptB['x']) ** 2 +
            (ptA['y'] - ptB['y']) ** 2
        )
        return res

    def slide(self, ring_pts, min_angle=0.00, max_angle=3.14):
        if ring_pts.shape[0] < 2:
            # self.get_logger().warn("Received ring with < 2 points, skipping")
            return

        # Initial distance is between first two points
        dist: float = self.dist_2d(ring_pts[0], ring_pts[1])
        biggest_dist = 0.0
        # Initial state is no curb
        prev_point = ring_pts[0]
        
        for pt in ring_pts[1:]:
            dist = self.dist_2d(pt, prev_point)
            abs_angle = abs(math.atan2(pt['y'],pt['x']))
            if dist > self.DIST_TOLERANCE or abs_angle > max_angle or abs_angle < min_angle:
                # self.get_logger().info("Dist={}, angle={}, {}".format(dist, abs_angle, pt))
                return prev_point
            if dist > biggest_dist:
                dist = biggest_dist
            # print(dist)
            prev_point = pt
        print("No curb found. Max dist: {}, last: {}".format(biggest_dist, ring_pts[:-1]))

    def searchRingForCurbs(self, ring_pts):
        # Find index of point at theta=0
        angles = np.arctan2(ring_pts['y'],ring_pts['x']) + math.sin(self.lidar_to_bl_tf.transform.rotation.z)
        # self.get_logger().info("Avg angle{}".format(np.mean(angles)))
        abs_angles = np.abs(angles)
        zero_idx = np.argmin(abs_angles)
        # self.get_logger().info("{} @ {}".format(ring_pts[zero_idx], zero_idx))

        # ring_pts = ring_pts[angles>0]
        # angles = angles[angles>0]
        left_pts = ring_pts[:zero_idx]
        left_curb_pt = self.slide(left_pts[::-1])
        # if pt is not None:
        #     plt.scatter(-pt['y'], pt['x'], s=20)

        right_pts = ring_pts[zero_idx:]
        right_curb_pt = self.slide(right_pts[::-1])
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
        curb_bounds = []

        for ring in range(self.TOP_RING):
            ring_pts = pts[pts['ring']==ring]
            bounds = self.searchRingForCurbs(ring_pts)
            for i in range(len(bounds)):
                if bounds[i] is not None:
                    curb_bounds.append(bounds[i])

        return curb_bounds

    def formPointCloud2(self, nparray, frame_id: str):
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, nparray)
        filtered_msg.header.frame_id = frame_id
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg

    def front_lidar_cb(self, msg: PointCloud2):
        if not self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
            return
        else: 
            self.getLidarToBlTransform('lidar_front')
            pts = rnp.numpify(msg)
            pts = self.filterPoints(pts, min_angle=0.00, max_angle=3.14)
            # self.publishCloud(pts, msg.header.frame_id)
            self.curb_candidates_pub.publish(self.formPointCloud2(pts, 'base_link'))

            curb_bounds = self.findAllCurbBounds(pts)
            np_bounds = np.array(curb_bounds, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint8)
            ])
            res_msg: PointCloud2 = rnp.msgify(PointCloud2, np_bounds)
            res_msg.header.frame_id = 'base_link'
            res_msg.header.stamp = self.get_clock().now().to_msg()
            self.curb_pts_pub.publish(res_msg)

    def process_lidar(self, msg: PointCloud2, lidar_frame: str):
        self.get_logger().info("{}".format("PROCESSING"))
        # Transform to base_link
        # if not self.tf_buffer.can_transform('base_link', lidar_frame, self.get_clock().now()):
        #     return
        # t_zero = rclpy.time.Time(seconds=0, nanoseconds=0)
        # lidar_to_bl_tf: TransformStamped = self.tf_buffer.lookup_transform('base_link', lidar_frame, t_zero)
        # self.get_logger().info(str(lidar_to_bl_tf.transform.translation.x))

        npcloud = rnp.numpify(msg)
        # if not exists('foo2.npy'):
        #     np.save('foo2.npy', npcloud)
        # self.get_logger().info(str(npcloud.shape))
        # Choose only points in ring 4 and below
        mask = npcloud[:]['ring'] <= self.TOP_RING
        npcloud = npcloud[mask]
        translation = lidar_to_bl_tf.transform.translation
        # npcloud['x'] += translation.x
        # npcloud['y'] += translation.y
        # npcloud['z'] += translation.z

        # Rotate each point
        quat = [
            lidar_to_bl_tf.transform.rotation.x,
            lidar_to_bl_tf.transform.rotation.y,
            lidar_to_bl_tf.transform.rotation.z,
            lidar_to_bl_tf.transform.rotation.w
        ]
        r = Rotation.from_quat(quat)
        xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        
        pts = r.apply(xyz)
        t = np.array([translation.x, translation.y, translation.z])
        np.add(pts, np.transpose(t), out=pts)
        
        # self.get_logger().info(str(xyz[:,0]))
        npcloud['x'] = pts[:,0]
        npcloud['y'] = pts[:,1]
        npcloud['z'] = pts[:,2]

        # At this point our data is now in the base_link frame.
        # Let's remove all points with z > 0.5 (meter)
        mask = npcloud[:]['z'] <= 0.5
        npcloud = npcloud[mask]
        slope_abs = -999.9

        # OK! Now find average height of points around car
        # Specifically, x=[-4,8] meters and y=[-1,2] meters
        mask = np.logical_and(
            np.logical_and(npcloud[:]['x'] > -4, npcloud[:]['x'] < 8),
            np.logical_and(npcloud[:]['y'] > -1, npcloud[:]['y'] < 2)
        )
        roadpoints = npcloud[mask]
        # for ring_number in range(self.TOP_RING):
        #     for point in roadpoints[roadpoints['ring'] == ring_number]:
        #         # if slope_abs == -999.9

        '''
        PSEUDOCODE

        1. Find average z in front of and behind car.

        2. For each ring from 0-4:
            a. For each point:
                i. If point.z > road_z + 0.1 m (4 inches), point is ABOVE_STREET
                ii. Elif point.z <= road_z + 0.1 m, ON_STREET
                    iia. If point.z < road_z - 0.1, WARN. Pothole?

            b. If previous state!=current_state, add point to CURB list
        '''
        # road_z = np.mean(roadpoints[:]['z'])

        # curb_pts = np.array([], dtype=[
        #     ('x', np.float32),
        #     ('y', np.float32),
        #     ('z', np.float32),
        #     ('intensity', np.float32),
        #     ('ring', np.uint8)
        # ])

        
        # TOLERANCE = 0.1 # meters
        # road_pts = npcloud[npcloud['z'] < road_z + TOLERANCE]
        curb_pts = []
        prev_point = npcloud[0]
        distMatrix = [[]]

        for ring_number in range(self.TOP_RING):
            currentState = RoadPointType.ON_ROAD
            ringDists = []
            ring_pts = npcloud[npcloud['ring'] == ring_number][1:]
            for idx, point in enumerate(ring_pts):
                dist = math.sqrt(
                    (point['x']-ring_pts[idx-1]['x']) ** 2 +
                    (point['y']-ring_pts[idx-1]['y']) ** 2 +
                    (point['z']-ring_pts[idx-1]['z']) ** 2
                )
                ringDists.append(dist)
            distMatrix.append(ringDists)

        curb_pt_array = np.array(curb_pts, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint8)
        ])  
        # self.get_logger().info("{}".format(distMatrix))
        np_dist_matrix = np.array(distMatrix)
        # self.get_logger().info("{}".format(np_dist_matrix))
        if not exists('foo1.npy'):
            np.save('foo1.npy', np_dist_matrix)
        
        
        # Construct message from numpy array and publish
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, curb_pt_array)
        filtered_msg.header.frame_id = 'base_link'
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg
        

    # def calibrate(self, points):
    #     # Find angle of point to LEFT or RIGHT (roll)
    #         # Points along Y axis will have X value close to 0, ring=3
    #     # Take abs value of all points' x values. Find min.
        
    #     ring_points = np.array([[]])
    #     for point in points:
    #         if point[3] == 4:
    #             ring_points = np.append(ring_points, [point], axis=1)
    #     abs_points = np.abs(ring_points)
    #     res = np.argmin(abs_points[0])
    #     self.get_logger().info("Min is: %s" % str(ring_points))

    #     # Then find angle of point IN FRONT (pitch)
    #         # Y value = 0, ring = 3



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