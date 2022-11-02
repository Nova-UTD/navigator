import numpy as np
import pickle
import sys
import os
import math
import grid_cell

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp

class DynamicGridConstructor(Node):
	def __init__(self):
		self.dynamic_grid = []
	def create_grid():
		for  i in range(1,100):
			temp_cell = grid_cell.GridCell()
			self.dynamic_grid.append(temp_cell)

def main(args=None):
	rclpy.init(args=args)
	dynamic_grid = DynamicGridConstructor()
	rclpy.spin(dynamic_grid)
	dynamic_grid.destroy_node()
	rclpy.shutdown()
