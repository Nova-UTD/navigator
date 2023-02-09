import numpy as np
import pickle
import sys
import os
import math
import serial
#from shadowcasting import ShadowCaster
from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from std_msgs.msg import String




class ThrottleNode(Node):
	def __init__(self):
		super().__init__('throttle_node')
		self.get_logger().info('#####################################################################################')
		self.carla_sub = self.create_subscription(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.send_message, 10)


    def send_message(self, control_data: CarlaEgoVehicleControl):
        # self.get_logger().info(str(control_data))
		with serial.Serial('/dev/ttyS1', 115200) as ser:
			ser.write(control_data)




def main(args=None):
	rclpy.init(args=args)
	throttle = ThrottleNode()
	rclpy.spin(throttle)
	#self.get_logger().info('#####################################################################################')
	throttle.destroy_node()
	rclpy.shutdown()
