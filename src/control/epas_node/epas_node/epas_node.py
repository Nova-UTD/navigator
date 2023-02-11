import numpy as np
import pickle
import sys
import os
import can
import math
from rosgraph_msgs.msg import Clock


from carla_msgs.msg import CarlaEgoVehicleControl
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nova_msgs.msg import StaticGrid
from nova_msgs.msg import GridRow

class can_message_constructor:
    torque = None
    motor_duty = None
    current = None
    supply_voltage = None
    switch_position = None
    box_temp = None
    torque_a = None
    torque_b = None
    steering_angle = None
    analog_c1 = None
    analog_c2 = None
    selected_map = None
    error_msg = None
    io_bf = None
    status_bf = None
    limit_bf = None
    def __init__(self,msg1,msg2):
        self.torque = msg1[0]
        self.motor_duty = msg1[1]
        self.current = msg1[2]
        self.supply_voltage = msg1[3]
        self.switch_position = msg1[4]
        self.box_temp = msg1[5]
        self.torque_a = msg1[6]
        self.torque_b = msg1[7]
        self.steering_angle = msg2[0]
        self.analog_c1 = msg2[1]
        self.analog_c2 = msg2[2]
        self.selected_map = msg2[3]
        self.error_msg = msg2[4]
        self.io_bf = msg2[5]
        self.status_bf = msg2[6]
        self.limit_bf = msg2[7]
    def to_string(self):
        return f"{self.torque},{self.motor_duty},{self.current},{self.supply_voltage/10},{self.switch_position},{self.box_temp},{self.torque_a},{self.torque_b},{self.steering_angle},{self.analog_c1},{self.analog_c2},{self.selected_map},{self.error_msg},{self.io_bf},{self.status_bf},{self.limit_bf}\n"

class epas_node(Node):
    target_angle = 0.0
    torque_a = 0
    torque_b = 0
    left_limit = 19
    right_limit = 230
    can_msg = None
    k_p = 1.0
    error = 0.0
    torque = []

    def __init__(self):
        super().__init__('epas_node')
        self.target_angle = self.create_subscription(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.receive_can_msg, 10)
        #self. = self.create_publisher(array, 'static_grid', 10)

    def send_can_msg(self):
        bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000, receive_own_messages=True)
        torque_a = 100
        torque_b = 155
        #data = [0x03, torque_a, torque_b, 0x00, 0x00, 0x00, 0x00, 0x00]
        #var = 0
        #**********8navigator root, pip dependencies*************
        i=0
        while(True):
            if i%2==0:
                data = [0x03, torque_a, torque_b, 0x00, 0x00, 0x00, 0x00, 0x00]
                message = can.Message(arbitration_id=0x296, data=data, check=True, is_extended_id=False)
                bus.send(message, timeout=0.2)
                i+=1
            else:
                data = [0x03, torque_b, torque_a, 0x00, 0x00, 0x00, 0x00, 0x00]
                message = can.Message(arbitration_id=0x296, data=data, check=True, is_extended_id=False)
                bus.send(message, timeout=0.2)
                i+=1
            
        '''
        torque_a = 155
        torque_b = 100
        data = [0x03, torque_a, torque_b, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x296, data=data, check=True, is_extended_id=False)
        bus.send(message, timeout=0.2)
        '''

    def receive_can_msg(self, msg: CarlaEgoVehicleControl):
        self.target_angle = msg.steer
        bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000, receive_own_messages=True)
        cached_msg1 = None
        while(True):
            msg = bus.recv(0.2)
            if (msg is None):
                print("Skipping")
            elif (msg.arbitration_id == 0x290):
                cached_msg1 = msg.data                    
            elif(msg.arbitration_id == 0x292):
                if (cached_msg1 is not None):
                        self.parse_can(cached_msg1, msg.data, bus)

    def parse_can(self, msg1_data: bytearray, msg2_data: bytearray, bus) -> str:
        #bus = can.interface.Bus(bustype='slcan', channel='/dev/tty.usbmodem14201', bitrate=500000, receive_own_messages=True)
        self.can_msg = can_message_constructor(msg1_data,msg2_data)
        print(self.can_msg.to_string())
        current_angle = self.can_msg.steering_angle
        current_angle_norm = ((current_angle-self.left_limit)/(self.right_limit-self.left_limit)*2)-1
        self.error = self.target_angle-current_angle_norm
        self.torque_a = int(min(255,max(0, math.ceil((self.error+1) * (255/2)))))
        self.torque_b = 255-self.torque_a

        data = [0x03, self.torque_a, self.torque_b, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x296, data=data, check=True, is_extended_id=False)
        bus.send(message, timeout=0.2)


def main(args=None):
	rclpy.init(args=args)
	EPAS = epas_node()
	rclpy.spin(EPAS)
	#self.get_logger().info('#####################################################################################')
	dynamic_grid.destroy_node()
	rclpy.shutdown()
