import numpy as np
import pickle
import sys
import os
import math
from rosgraph_msgs.msg import Clock


from carla_msgs.msg import CarlaEgoVehicleControl
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nova_msgs.msg import StaticGrid
from nova_msgs.msg import GridRow



class joy_interface_node(Node):
    target_joy = 0.0
    output_msg = None

    def __init__(self):
        super().__init__('joy_interface_node')
        self.target_joy = self.create_subscription(Joy, '/joy',self.process_joy, 10)
        self.output_msg = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        #self. = self.create_publisher(array, 'static_grid', 10)

    def process_joy(self, joy_msg: Joy):
        self.get_logger().info(str(joy_msg))
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = Clock().clock
        msg.header.frame_id = 'base_link'
        msg.throttle = joy_msg.axes[2]*-1
        msg.steer = joy_msg.axes[0]*-1
        msg.brake = joy_msg.axes[5]*-1
        msg.hand_brake = True if joy_msg.buttons[2]==1 else False
        msg.reverse = True if joy_msg.buttons[0]==1 else False
        msg.gear = True if joy_msg.buttons[3]==1 else False
        msg.manual_gear_shift = True if joy_msg.buttons[1]==1 else False
        self.output_msg.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    JOYNODE = joy_interface_node()
    rclpy.spin(JOYNODE)
    #self.get_logger().info('#####################################################################################')
    joy_interface_node.destroy_node()
    rclpy.shutdown()
