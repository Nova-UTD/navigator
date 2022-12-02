import numpy
import os
import math
import time

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp

class GridCell(Node):
	def __init__(self):
		self.x = 0
		self.y = 0
		self.m_free = m_free
	        # posterior occupied mass
       	        self.m_occ = m_occ
	        # normalization component for associated measurements
       	        self.mu_A = 1.
	        # normalization component for unassociated measurements
                self.mu_UA = 0.
	        # statistical moments: velocity
                self.mean_x_vel = 0.
                self.mean_y_vel = 0.
                self.var_x_vel = 0.
                self.var_y_vel = 0.
                self.covar_xy_vel = 0.
        	# statistical moments: acceleration
        	self.mean_x_acc = 0.
        	self.mean_y_acc = 0.
        	self.var_x_acc = 0.
        	self.var_y_acc = 0.
        	self.covar_xy_acc = 0.
