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
from voltron_msgs.msg import StaticGrid
from voltron_msgs.msg import GridRow



class ShadowCaster:

    def __init__(self, mapwidth, mapheight):
        self.width = mapwidth
        self.height = mapheight
        self.visiblemap = []
        self.seethrough = []
        self.sourcex = 0
        self.sourcey = 0
        self.range = 0

    def cast_shadow(self, seethrough, sourcex, sourcey, sightrange):
        self.sourcex = sourcex
        self.sourcey = sourcey
        self.range = sightrange
        self.seethrough = seethrough
        self.visiblemap = [[False for y in range(self.height)]
                           for x in range(self.width)]
        self.visiblemap[sourcex][sourcey] = True
        for octant in range(1, 9):
            self._scan(1, octant, 1.0, 0.0)
        return self.visiblemap

    def _scan(self, depth, octant, startslope, endslope):
        x = 0
        y = 0
        if octant == 1:   # NW
            x = self.sourcex - int(startslope * depth)
            y = self.sourcey - depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x - 1, y, False):
                                startslope = self._get_slope(x - .5, y - .5,
                                                             self.sourcex,
                                                             self.sourcey)
                        else:
                            if self._test_tile(x - 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x - .5, y + .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x += 1
                x -= 1
        elif octant == 2:   # NE
            x = self.sourcex + int(startslope * depth)
            y = self.sourcey - depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x + 1, y, False):
                                startslope = -self._get_slope(x + .5, y - .5,
                                                              self.sourcex,
                                                              self.sourcey)
                        else:
                            if self._test_tile(x + 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x + .5, y + .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x -= 1
                x += 1
        elif octant == 3:   # EN
            x = self.sourcex + depth
            y = self.sourcey - int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y - 1, False):
                                startslope = -self._get_inv_slope(x + .5,
                                                                  y - .5,
                                                                  self.sourcex,
                                                                  self.sourcey)
                        else:
                            if self._test_tile(x, y - 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x - .5, y - .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y += 1
                y -= 1
        elif octant == 4:   # ES
            x = self.sourcex + depth
            y = self.sourcey + int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y + 1, False):
                                startslope = self._get_inv_slope(x + .5,
                                                                 y + .5,
                                                                 self.sourcex,
                                                                 self.sourcey)
                        else:
                            if self._test_tile(x, y + 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x - .5, y + .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y -= 1
                y += 1
        elif octant == 5:   # SE
            x = self.sourcex + int(startslope * depth)
            y = self.sourcey + depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x + 1, y, False):
                                startslope = self._get_slope(x + .5, y + .5,
                                                             self.sourcex,
                                                             self.sourcey)
                        else:
                            if self._test_tile(x + 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x + .5, y - .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x -= 1
                x += 1
        elif octant == 6:   # SW
            x = self.sourcex - int(startslope * depth)
            y = self.sourcey + depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x - 1, y, False):
                                startslope = -self._get_slope(x - .5, y + .5,
                                                              self.sourcex,
                                                              self.sourcey)
                        else:
                            if self._test_tile(x - 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x - .5, y - .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x += 1
                x -= 1
        elif octant == 7:   # WS
            x = self.sourcex - depth
            y = self.sourcey + int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y + 1, False):
                                startslope = -self._get_inv_slope(x - .5,
                                                                  y + .5,
                                                                  self.sourcex,
                                                                  self.sourcey)
                        else:
                            if self._test_tile(x, y + 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x + .5, y + .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y -= 1
                y += 1
        elif octant == 8:   # WN
            x = self.sourcex - depth
            y = self.sourcey - int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y - 1, False):
                                startslope = self._get_inv_slope(x - .5,
                                                                 y - .5,
                                                                 self.sourcex,
                                                                 self.sourcey)
                        else:
                            if self._test_tile(x, y - 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x + .5, y - .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y += 1
                y -= 1
        if x < 0:
            x = 0
        if x >= self.width:
            x = self.width - 1
        if y < 0:
            y = 0
        if y >= self.height:
            y = self.height - 1
        if self._is_visible(x, y) and self.seethrough[x][y]:
            self._scan(depth + 1, octant, startslope, endslope)

    def _check_bounds(self, x, y):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        else:
            return True

    def _get_slope(self, x1, y1, x2, y2):
        return float(x1 - x2) / float(y1 - y2)

    def _get_inv_slope(self, x1, y1, x2, y2):
        return float(y1 - y2) / float(x1 - x2)

    def _is_visible(self, x, y):
        if self._check_bounds(self.sourcex, self.sourcey) and \
                self._check_bounds(x, y):
            return math.hypot(self.sourcex - x, self.sourcey - y) <= self.range
        else:
            return False

    def _test_tile(self, x, y, state):
        if not self._is_visible(x, y):
            return False
        else:
            return self.seethrough[x][y] == state


class DynamicGridNode(Node):
	def __init__(self):
		super().__init__('dynamic_grid_node')
		self.dynamic_grid = []
		self.RESULT_PATH = './'
		self.point_cloud_sub = self.create_subscription(PointCloud2, 'ground_seg_points', self.create_grid, 10)
		self.grid_pub = self.create_publisher(OccupancyGrid, 'static_grid', 10)

		#self.point_cloud_pub = self.create_publisher(PointCloud2, 'ground_seg_points', 10)


	def create_grid(self, point_cloud: PointCloud2):

		#df = pd.read_csv('input.csv')
		data = rnp.numpify(point_cloud)
		#pcd = o3d.geometry.PointCloud()
		#pcd.points = o3d.utility.Vector3dVector(data)
		#arr = np.asarray(pcd.points)
		#print(arr.size)
		occ = self.pcd_to_sensor_grid(data)

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
		'''
		StaticGrid grid
		for row in static_occ:
			GridRow tempRow
			for value in row:
				tempRow.append(value)
			grid.append(tempRow)
		'''
		#self.grid_pub.publish(grid)
		msg = OccupancyGrid()
		msg.header = Clock().clock
		msg.map_load_time = Clock().clock
		msg.info.resolution = 0.4
		msg.info.width = width
		msg.info.height = height
		msg.data = static_occ.flatten()

		self.grid_pub.data.publish(msg)
		return static_occ
	
	def pcd_flattener(points: np.ndarray):
		for point in points:
			points[2] = 0
		return points
	
	def pcd_to_sensor_grid(self, points: np.ndarray):
		rows = 120
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

	
	#def grid_to_image(arr: np.ndarray, path: str):
    	#cv2.imwrite(f'{path}.png',(arr * 255).astype(np.uint8))



def main(args=None):
	rclpy.init(args=args)
	dynamic_grid = DynamicGridNode()
	rclpy.spin(dynamic_grid)
	dynamic_grid.destroy_node()
	rclpy.shutdown()
