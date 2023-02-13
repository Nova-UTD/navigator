'''
Package: grids
   File: grid_summation_node.py
 Author: Will Heitman (w at heit dot mn)

Subscribes to cost maps, calculates their weighted sum, and
publishes the result as a finished cost map.
'''

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
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


class GridSummationNode(Node):

    def __init__(self):
        """Subscribe to the desired cost maps

        - Drivable surface

        """
        super().__init__('grid_summation_node')

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/carla/hero/lidar', self.lidar_cb, 10)

        self.semantic_lidar_sub = self.create_subscription(
            PointCloud2, '/carla/hero/semantic_lidar', self.semantic_lidar_cb, 10)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )
        self.clean_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/fused', 10
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
        # msg_array = self.remove_points_above(msg_array, 2.0)
        # msg_array = self.remove_ground_points(msg_array, 0.2)

        # self.publish_occupancy_grid(msg_array, range=40.0, res=0.5)

        merged_pcd_msg: PointCloud2 = rnp.msgify(PointCloud2, msg_array)
        merged_pcd_msg.header.frame_id = 'base_link'
        merged_pcd_msg.header.stamp = self.carla_clock.clock

        self.clean_lidar_pub.publish(merged_pcd_msg)

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


def main(args=None):
    rclpy.init(args=args)

    grid_summation_node = GridSummationNode()

    rclpy.spin(grid_summation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    grid_summation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
