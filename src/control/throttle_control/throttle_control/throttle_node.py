import numpy as np
import pickle
import sys
import os
import math
import serial
import io
#from shadowcasting import ShadowCaster

# Message definitions
from carla_msgs.msg import CarlaEgoVehicleControl
from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from std_msgs.msg import String




class throttle_node(Node):
    def __init__(self):
        super().__init__('throttle_node')
        self.get_logger().info('#####################################################################################')
        self.carla_sub = self.create_subscription(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.send_message, 10)

        test_msg = CarlaEgoVehicleControl()
        test_msg.throttle = 0.5

        self.send_message(test_msg)
        self.send_message(test_msg)


    def send_message(self, msg: CarlaEgoVehicleControl):
        self.get_logger().info(f"Received throttle with value {msg.throttle}")

        throttle = msg.throttle
        with serial.Serial('/dev/ttyACM0', 115200) as ser:
            # cmd = f"t{throttle}".encode('utf-16be')
            sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
            sio.write(u't0.5\n')
            sio.flush()
            hello = sio.readline()
            print(hello)




def main(args=None):
    rclpy.init(args=args)
    throttle = throttle_node()
    rclpy.spin(throttle)
    #self.get_logger().info('#####################################################################################')
    throttle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
