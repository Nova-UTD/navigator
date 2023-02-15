'''
Package:   parade_controller
Filename:  controller.py
Author:    Daniel Vayman

Controller for the hoco parade that follows our flag
'''

from rosgraph_msgs.msg import Clock
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

import rclpy
import ros2_numpy as rnp
import numpy as np

class ParadeController(Node):

    def __init__(self):
        super().__init__('parade_controller_node')

        self.INTENSITY_THRESHOLD = 0.75

        # Subscriber
        self.lidar_left_sub = self.create_subscription(PointCloud2, 'velodyne_points', self.pointclouds_cb, 10)

        # Publisher
        self.throttle_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
    
    def pointclouds_cb(self, msg: PointCloud2)
        pcd_temp: np.array = rnp.numpify(msg)
        pcd: np.array

        # Logic for combining 2 point clouds into pcd, maybe holding on to pointclouds until first point intensity is differet?
        #
        #
        #

        # Removes all points below the intensity threshold
        pcd = pcd[pcd[:, 3] > self.INTENSITY_THRESHOLD]

        # Finds the center of mass of banner points, resulting in a 1D numpy array (x, y) with the center coordinate of the banner
        banner = np.mean(pcd[:, :2], axis = 0)

        # TODO: Compute the distance between (0, 0) and the banner coordinate

        # TODO: Controller logic

        # TODO: Set msg fields and publish throttle value


        

def main(args=None):
    rclpy.init(args=args)

    parade_controller_node = ParadeController()

    rclpy.spin(parade_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parade_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
