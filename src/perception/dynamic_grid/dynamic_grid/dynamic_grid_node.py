import numpy as np
import open3d as o3d
import pandas as pd
import cv2
import pickle
import sys
import os
import math
import grid_cell
from shadowcasting import ShadowCaster

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from voltron_msgs.msg import StaticGrid
from voltron_msgs.msg import GridRow


class DynamicGridNode(Node):
	def __init__(self):
		super().__init__('dynamic_grid_node')
		self.dynamic_grid = []
		self.RESULT_PATH = './'
		self.point_cloud_sub = self.create_subscription(PointCloud2, 'ground_seg_points', self.create_grid, 10)
		self.grid_pub = self.create_publisher(StaticGrid, 'static_grid', 10)

		#self.point_cloud_pub = self.create_publisher(PointCloud2, 'ground_seg_points', 10)


	def create_grid(self, point_cloud: PointCloud2):

		for  i in range(1,100):
			temp_cell = grid_cell.GridCell()
			self.dynamic_grid.append(temp_cell)

		#df = pd.read_csv('input.csv')
		data = rnp.numpify(point_cloud)
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(data)
		arr = np.asarray(pcd.points)
		print(arr.size)
		occ = self.pcd_to_sensor_grid(arr)

		width = 120
		height = 120
		fov = ShadowCaster(width, height)
		for y in range(height):
			for x in range(width):
				if x == 0 or y == 0 or x == width - 1 or y == height - 1:
					occ[x][y] = False

		
		result = fov.cast_shadow(occ, 59, 59, 240)
		static_occ = np.zeros([height, width])
		for y in range(height):
			for x in range(width):
				if result[x][y]:
					if occ[x][y]:
						static_occ[x][y] = 0
					else:
						static_occ[x][y] = 1
				else:
					static_occ[x][y] = 1

		StaticGrid grid
		for row in static_occ:
			GridRow tempRow
			for value in row:
				tempRow.append(value)
			grid.append(tempRow)
		
		self.grid_pub.publish(grid)
		
		return static_occ
	
	def pcd_flattener(points: np.ndarray):
		for point in points:
			points[2] = 0
		return points
	
	def pcd_to_sensor_grid(self, points: np.ndarray):
		rows = 80
		cols = 120
		#this grid creates the base for the egma we will send over to BPC
		sensor_grid = [[True for i in range(cols)] for j in range(rows)]
		for point in points:
			x_cor = point[0]
			y_cor = point[1]
			x_cor = math.ceil(x_cor)
			y_cor = math.ceil(y_cor)
			sensor_grid[x_cor][y_cor] = False
		
		return sensor_grid
		#return np.array(sensor_grid)

	
	def grid_to_image(arr: np.ndarray, path: str):
    	cv2.imwrite(f'{path}.png',(arr * 255).astype(np.uint8))



def main(args=None):
	rclpy.init(args=args)
	dynamic_grid = DynamicGridNode()
	rclpy.spin(dynamic_grid)
	dynamic_grid.destroy_node()
	rclpy.shutdown()
