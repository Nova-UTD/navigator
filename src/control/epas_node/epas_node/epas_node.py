import numpy as np
import pickle
import sys
import os
import can
import math
from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nova_msgs.msg import StaticGrid
from nova_msgs.msg import GridRow






def main(args=None):
	rclpy.init(args=args)
	dynamic_grid = DynamicGridNode()
	rclpy.spin(dynamic_grid)
	#self.get_logger().info('#####################################################################################')
	dynamic_grid.destroy_node()
	rclpy.shutdown()
