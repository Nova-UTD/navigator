import numpy as np
import pickle
import sys
import os
import math
#from shadowcasting import ShadowCaster
from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nova_msgs.msg import StaticGrid
from nova_msgs.msg import GridRow

class ParadeControllerNode(Node):
	def __init__(self):
		super().__init__('parade_controller_node')

		#self.point_cloud_sub = self.create_subscription(PointCloud2, '/lidar_filtered', self.create_grid, 10)
		
		#self.grid_pub = self.create_publisher(OccupancyGrid, 'static_grid', 10)


def main(args=None):
	rclpy.init(args=args)
	parade_controller = ParadeControllerNode()
	rclpy.spin(parade_controller)
	parade_controller.destroy_node()
	rclpy.shutdown()
