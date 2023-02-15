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
import ros2_numpy as np

class ParadeController(Node):
    def __init__(self):
        super().__init__('parade_controller_node')

        # Subscriber
        self.lidar_left_sub = self.create_subscription(PointCloud2, 'velodyne_points', self.pointclouds_cb, 10)

        # Publisher
        self.throttle_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
    
    def pointclouds_cb(self, msg: PointCloud2)
        
        

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
