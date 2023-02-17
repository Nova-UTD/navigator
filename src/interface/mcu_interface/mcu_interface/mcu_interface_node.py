'''
Package:   mcu_interface
Filename:  mcu_interface_node.py
Author:    Will Heitman (w at heit.mn)

Subscribes to CarlaEgoVehicleControl messages (https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaEgoVehicleControl.msg)

Sends the appropriate brake data to the LA
'''

import os
import random
import time
import io

import serial
import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

bus = None
COMMAND_ID = 0xFF0000
REPORT_ID = 0xFF0001


class McuInterfaceNode(Node):

    def __init__(self, bus):
        super().__init__('mcu_interface_node')

        self.last_command_rcv_time = None

        self.vehicle_command_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.commandCb, 1)

        self.brake = 0.0  # 0.0 is released, 1.0 is fully pressed
        self.bus: serial.Serial = bus

        while bus is None:
            self.get_logger().warn("Bus not yet set. Waiting...")
            time.sleep(1.0)

        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.bus, self.bus))

        self.get_logger().info("Bus now connected.")

        self.vehicle_command_timer = self.create_timer(0.5, self.publishCommand)
        self.throttle = 0

    def commandCb(self, msg: CarlaEgoVehicleControl):
        self.throttle = msg.throttle
        self.last_command_rcv_time = time.time()

    def publishCommand(self):
        if self.last_command_rcv_time is None:
            return
        

        throttle = self.throttle
        if time.time() - self.last_command_rcv_time > 0.5:
            throttle = 0.0
        # if throttle < 0.1:
        #     return
        throttle = min(throttle, 0.3)
        self.get_logger().info(f"Throttle = {throttle}")

        command = str.encode(f"{throttle}\r\n")
        self.bus.write(command)
        # self.bus.write(b"0.7\r\n")

        # self.sio.write(f"$throttle,{throttle};\n")

        # response = self.sio.readline()
        # self.get_logger().info(response)
        # self.sio.flush()
    

def main(args=None):
    rclpy.init(args=args)

    # Try ls /dev/seria/by-id
    channel = '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Grand_Central_M4_Express_BA3E99D033533853202020350D3B12FF-if00'
    bitrate = 115200

    bus = None

    while bus is None:
        try:
            bus = serial.Serial(channel, bitrate, timeout=1)
        except Exception as e:
            print(f"An error occured opening the bus: {e} Retrying...")
            time.sleep(1.0)

    mcu_interface_node = McuInterfaceNode(bus)

    rclpy.spin(mcu_interface_node)

    # Close the serial connection
    command = str.encode(f"0.0\r\n")
    bus.write(command)
    bus.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mcu_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
