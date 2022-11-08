import numpy as np
import open3d as o3d
import pandas as pd
import cv2
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
		self.RESULT_PATH = './'
	def create_grid():
		for  i in range(1,100):
			temp_cell = grid_cell.GridCell()
			self.dynamic_grid.append(temp_cell)

		df = pd.read_csv('input.csv')
		data = df.to_numpy()
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(data)
		arr = np.asarray(pcd.points)
		print(arr.size)
		occ = pcd_to_occupancy_grid(arr)
		print(type(occ))
		print(occ.size)
		numpy_to_image(occ, os.path.join(RESULT_PATH, 'image'))
	
	def grid_to_image(arr: np.ndarray, path: str):
    	cv2.imwrite(f'{path}.png',(arr * 255).astype(np.uint8))

	def pcd_to_occupancy_grid(pcd: np.ndarray):
    
		pcd = pcd.round()

		x_min = pcd[:, 0].min()
		x_max = pcd[:, 0].max()

		y_min = pcd[:, 1].min()
		y_max = pcd[:, 1].max()


		occ = np.zeros((int(x_max - x_min + 1), int(y_max - y_min + 1)), dtype=np.float64)

		for i in range(pcd.shape[0]):
			x = int(pcd[i, 0] - x_min)
			y = int(pcd[i, 1] - y_min)

			occ[x, y] += 1

		occ /= pcd.shape[0]
		return occ


def main(args=None):
	rclpy.init(args=args)
	dynamic_grid = DynamicGridConstructor()
	rclpy.spin(dynamic_grid)
	dynamic_grid.destroy_node()
	rclpy.shutdown()
