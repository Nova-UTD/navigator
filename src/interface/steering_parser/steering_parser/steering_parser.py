#!/usr/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32

class SteeringParserNode(Node):

    def __init__(self):
        super().__init__('steering_parser')

        self.MIN_READING = 0
        self.MAX_READING = 1024
        self.MAX_ANGLE = 0.506

        self.angle_pub = self.create_publisher(
            Float32, '/real_steering_angle', 10)

        self.string_data_sub = self.create_subscription(
            StringMsg, '/serial_incoming_lines', self.string_data_callback, 10)

    def string_data_callback(self, msg):
        line = msg.data
        if (line == ""):
            return
        parts = line.split()
        # print(parts)

        if(parts[0] != "steering"):
            return
        
        factor = (float(parts[1]) - self.MIN_READING) / (self.MAX_READING - self.MIN_READING)
        angle = ((factor * 2) - 1) * self.MAX_ANGLE

        msg = Float32()
        msg.data = angle
        self.angle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SteeringParserNode()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
