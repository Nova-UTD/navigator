'''
Package: sensor_processing
   File: lidar_processing_node.py
 Author: Will Heitman (w at heit dot mn)

Node to filter and process raw LiDAR pointclouds

This node specifically deals with quirks with the
CARLA simulator. Namely, synching issues mean that
raw LiDAR streams "flicker" from left-sided PCDs
to right-sided ones. This node first merges
these left- and right-sided PCDs into a complete
cloud before cutting out points near the car.
'''

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

import matplotlib.pyplot as plt


class LidarProcessingNode(Node):

    def __init__(self):
        super().__init__('lidar_processing_node')
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/carla/hero/lidar', self.lidar_cb, 10)

        self.semantic_lidar_sub = self.create_subscription(
            PointCloud2, '/carla/hero/semantic_lidar', self.semantic_lidar_cb, 10)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )
        self.clean_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar_filtered', 10
        )

        self.clean_semantic_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar_semantic_filtered', 10
        )

        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, '/cost/occupancy', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.carla_clock = Clock()
        self.left_pcd_cached = np.zeros(0, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])
        self.right_pcd_cached = np.zeros(0, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])

        self.left_sem_pcd_cached = np.zeros(0, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        self.right_sem_pcd_cached = np.zeros(0, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])

    def clock_cb(self, msg: Clock):
        self.carla_clock = msg

    def lidar_cb(self, msg: PointCloud2):
        pcd_array: np.array = rnp.numpify(msg)

        min_y = np.min(pcd_array['y'])
        max_y = np.max(pcd_array['y'])

        if (min_y < -5.0):
            # This PCD is from the right side of the car
            # (Recall that +y points to the left)
            self.right_pcd_cached = pcd_array
        elif (max_y > 5.0):
            self.left_pcd_cached = pcd_array
        else:
            self.get_logger().warning('Incoming LiDAR PCD may be invalid.')

        merged_x = np.append(
            self.left_pcd_cached['x'], self.right_pcd_cached['x'])
        merged_y = np.append(
            self.left_pcd_cached['y'], self.right_pcd_cached['y'])
        merged_z = np.append(
            self.left_pcd_cached['z'], self.right_pcd_cached['z'])
        merged_i = np.append(
            self.left_pcd_cached['intensity'], self.right_pcd_cached['intensity'])

        total_length = self.left_pcd_cached['x'].shape[0] + \
            self.right_pcd_cached['x'].shape[0]

        msg_array = np.zeros(total_length, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])

        msg_array['x'] = merged_x
        msg_array['y'] = merged_y
        msg_array['z'] = merged_z
        msg_array['intensity'] = merged_i

        msg_array = self.transform_to_base_link(msg_array)
        msg_array = self.remove_nearby_points(msg_array, 3.0, 2.0)
        msg_array = self.remove_points_above(msg_array, 2.0)
        msg_array = self.remove_ground_points(msg_array, 0.2)

        self.publish_occupancy_grid(msg_array, range=40.0, res=0.5)

        merged_pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, msg_array)
        merged_pcd_msg.header.frame_id = 'base_link'
        merged_pcd_msg.header.stamp = self.carla_clock.clock

        self.clean_lidar_pub.publish(merged_pcd_msg)

    def publish_occupancy_grid(self, msg_array: np.array, range: float, res: float):
        start = time.time()

        grid = OccupancyGrid()
        pts_plain = np.zeros((msg_array.shape[0], 3))

        pts_plain[:, 0] = msg_array['x']
        pts_plain[:, 1] = msg_array['y']
        pts_plain[:, 2] = msg_array['z']

        # 1. First transform PCD to base_link
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform from "map" to "base_link": {ex}')
            return

        # 1a. Rotate all points
        q = t.transform.rotation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        pts_plain_tfed = r.apply(pts_plain)

        # 1b. Translate to complete the tf
        center = t.transform.translation
        trans_array = [center.x, center.y, center.z]
        pts_plain_tfed += trans_array

        min_x = center.x - range
        max_x = center.x + range
        min_y = center.y - range
        max_y = center.y + range
        width = np.ceil(max_x-min_x)
        height = np.ceil(max_y-min_y)

        angles = np.arctan2(pts_plain[:, 1], pts_plain[:, 0])

        # plt.hist(angles)
        # plt.show()
        # self.get_logger().info(f"Min: {angles.min()}, max: {angles.max()}")

        data =
        for i in np.linspace(min_x, max_x, res):
            for j in np.linspace(min_y, max_y, res):

                # for point in msg_array:
                #     self.get_logger().info(str(point))

        self.occupancy_grid_pub.publish(grid)

        self.get_logger().info(f"Grid gen took {time.time() - start} sec")

    def semantic_lidar_cb(self, msg: PointCloud2):
        pcd_array: np.array = rnp.numpify(msg)

        # Only keep ground points for now
        pcd_array = pcd_array[pcd_array['ObjTag'] == 7]

        min_y = np.min(pcd_array['y'])
        max_y = np.max(pcd_array['y'])

        if (min_y < -1.0):
            # This PCD is from the right side of the car
            # (Recall that +y points to the left)
            self.right_sem_pcd_cached = pcd_array
        elif (max_y > 1.0):
            self.left_sem_pcd_cached = pcd_array
        else:
            self.get_logger().warning('Incoming LiDAR PCD may be invalid.')

        merged_x = np.append(
            self.left_sem_pcd_cached['x'], self.right_sem_pcd_cached['x'])
        merged_y = np.append(
            self.left_sem_pcd_cached['y'], self.right_sem_pcd_cached['y'])
        merged_z = np.append(
            self.left_sem_pcd_cached['z'], self.right_sem_pcd_cached['z'])

        total_length = self.left_sem_pcd_cached['x'].shape[0] + \
            self.right_sem_pcd_cached['x'].shape[0]

        msg_array = np.zeros(total_length, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])

        msg_array['x'] = merged_x
        msg_array['y'] = merged_y
        msg_array['z'] = merged_z

        msg_array = self.transform_to_base_link(msg_array)

        merged_pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, msg_array)
        merged_pcd_msg.header.frame_id = 'base_link'
        merged_pcd_msg.header.stamp = self.carla_clock.clock

        self.clean_semantic_lidar_pub.publish(merged_pcd_msg)

    def transform_to_base_link(self, pcd: np.array) -> np.array:
        '''
        Transform input cloud into car's origin frame
        :param pcd: Original pcd in another coordinate frame (e.g. hero/lidar)
        :returns: Transformed pcd as a ros2_numpy array
        '''

        # TODO: Actually look up transform and perform translation
        #       and rotation accordingly.
        pcd['z'] += 2.08
        return pcd

    def remove_ground_points(self, pcd: np.array, height: float) -> np.array:
        '''
        Simple function to remove points below a certain height.
        :param pcd: a numpy array of the incoming point cloud, in the format provided by ros2_numpy.
        :param height: points with a z value less than this will be removed
        :returns: an array in ros2_numpy format with low points removed
        '''
        return pcd[pcd['z'] >= height]

    def remove_nearby_points(self, pcd: np.array, x_distance: float, y_distance: float) -> np.array:
        '''
        Remove points in a rectangle around the sensor

        :param pcd: a numpy array of the incoming point cloud, in the format provided by ros2_numpy.
        :param x_distance, y_distance: points with an x/y value whose absolute value is less than this number will be removed

        :returns: an array in ros2_numpy format with the nearby points removed
        '''

        # Ensure these positive
        x_distance = abs(x_distance)
        y_distance = abs(y_distance)

        # Unfortunately, I can't find a way to avoid doing this in one go
        pcd = pcd[np.logical_not(
            np.logical_and(
                np.logical_and(pcd['x'] > (x_distance*-1),
                               pcd['x'] < x_distance),
                np.logical_and(pcd['y'] > (y_distance*-1),
                               pcd['y'] < y_distance),
            ))]

        pcd = pcd[pcd['z'] >= -2.0]

        return pcd

    def remove_points_above(self, pcd: np.array, height: float) -> np.array:
        '''
        Remove points above the height of the sensor specified around the sensor

        :param pcd: a numpy array of the incoming point cloud, in the format provided by ros2_numpy.
        :param min_height: points with an z value whose absolute value is greater than this number will be removed

        :returns: an array in ros2_numpy format with the nearby points removed
        '''

        pcd = pcd[pcd['z'] <= height]

        return pcd


def main(args=None):
    rclpy.init(args=args)

    lidar_processor = LidarProcessingNode()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
