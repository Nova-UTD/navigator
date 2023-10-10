'''
Package: sensor_processing
   File: dual_lidar_processing_node.py
 Author: Will Heitman (w at heit dot mn)

Node to fuse, filter, and process LiDAR data from two sensors

This node is intended for real-world use.
'''

import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
import ros2_numpy as rnp
from scipy.spatial.transform import Rotation as R
# Message definitions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DualLidarProcessingNode(Node):

    def __init__(self):
        super().__init__('dual_lidar_processing_node')

        self.left_pcd_cached = None
        self.right_pcd_cached = None
        self.clock = 0.0
        self.clock_msg = Clock()

        self.left_lidar_sub = self.create_subscription(
            PointCloud2, '/velo_left/velodyne_points', self.leftLidarCb, 10)
        
        self.right_lidar_sub = self.create_subscription(
            PointCloud2, '/velo_right/velodyne_points', self.rightLidarCb, 10)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10
        )

        self.clean_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar', 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_recv_time = 0.0 # sec

    def clockCb(self, msg: Clock):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_msg = msg

    def leftLidarCb(self, msg: PointCloud2):
        """For the left LiDAR, simply convert it to Numpy at cache it.

        Args:
            msg (PointCloud2)
        """        
        self.left_pcd_cached: np.array = rnp.numpify(msg)
        self.left_pcd_cached = self.transformToBaseLink(self.left_pcd_cached, 'lidar_left')

        self.left_recv_time = self.clock

    def rightLidarCb(self, msg: PointCloud2):
        self.right_pcd_cached: np.array = rnp.numpify(msg)
        self.right_pcd_cached = self.transformToBaseLink(self.right_pcd_cached, 'lidar_right')

        # Check the time difference between the left and right PCDs
        right_lidar_recv_time = self.clock
        STALENESS_TOLERANCE = 0.5 

        
        if self.left_pcd_cached is None:
            return

        merged_x = np.append(
            self.left_pcd_cached['x'], self.right_pcd_cached['x'])
        merged_y = np.append(
            self.left_pcd_cached['y'], self.right_pcd_cached['y'])
        merged_z = np.append(
            self.left_pcd_cached['z'], self.right_pcd_cached['z'])
        merged_i = np.append(
            self.left_pcd_cached['intensity'], self.right_pcd_cached['intensity'])

        total_length = (self.left_pcd_cached.shape[0] + \
            self.right_pcd_cached.shape[0]) * 16 # 16 rings

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

        msg_array = self.remove_nearby_points(msg_array, (-4.0,0.5), (-1.3,1.3))
        # msg_array = self.remove_points_above(msg_array, 2.0)
        # msg_array = self.remove_ground_points(msg_array, 0.2)

        # self.publish_occupancy_grid(msg_array, range=40.0, res=0.5)

        merged_pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, msg_array)
        merged_pcd_msg.header.frame_id = 'base_link'
        merged_pcd_msg.header.stamp = self.clock_msg.clock

        self.clean_lidar_pub.publish(merged_pcd_msg)

    def transformToBaseLink(self, pcd: np.array, source_frame: str) -> np.array:
        '''
        Transform input cloud into car's origin frame
        :param pcd: Original pcd in another coordinate frame (e.g. hero/lidar)
        :returns: Transformed pcd as a ros2_numpy array
        '''

        try:
            t: TransformStamped = self.tf_buffer.lookup_transform('base_link', source_frame, rclpy.time.Time())
            quat = t.transform.rotation
        except Exception as e:
            self.get_logger().error(str(e))
            return


        # First rotate
        xyz = np.vstack([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T
        print(xyz)
        r: R = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        xyz = r.apply(xyz)

        pcd['x'] = xyz[:,0].reshape(-1,16)
        pcd['y'] = xyz[:,1].reshape(-1,16)
        pcd['z'] = xyz[:,2].reshape(-1,16)

        # Then translate
        pcd['x'] += t.transform.translation.x
        pcd['y'] += t.transform.translation.y
        pcd['z'] += t.transform.translation.z


        # TODO: Actually look up transform and perform translation
        #       and rotation accordingly.
        # pcd['z'] += 2.08
        return pcd

    def remove_ground_points(self, pcd: np.array, height: float) -> np.array:
        '''
        Simple function to remove points below a certain height.
        :param pcd: a numpy array of the incoming point cloud, in the format provided by ros2_numpy.
        :param height: points with a z value less than this will be removed
        :returns: an array in ros2_numpy format with low points removed
        '''
        return pcd[pcd['z'] >= height]

    def remove_nearby_points(self, pcd: np.array, x: tuple, y: tuple) -> np.array:
        '''
        Remove points in a rectangle around the sensor

        :param pcd: a numpy array of the incoming point cloud, in the format provided by ros2_numpy.
        :param x_distance, y_distance: points with an x/y value whose absolute value is less than this number will be removed

        :returns: an array in ros2_numpy format with the nearby points removed
        '''

        # Unfortunately, I can't find a way to avoid doing this in one go
        pcd = pcd[np.logical_not(
            np.logical_and(
                np.logical_and(pcd['x'] > x[0],
                               pcd['x'] < x[1]),
                np.logical_and(pcd['y'] > y[0],
                               pcd['y'] < y[1]),
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

    lidar_processor = DualLidarProcessingNode()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
