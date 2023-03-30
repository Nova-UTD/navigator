'''
Package:   clock
Filename:  clock_node.py
Author:    Will Heitman (w at heit.mn)

Publishes:
/clock (rosgraph_msgs/msg/Clock)
'''

from rosgraph_msgs.msg import Clock
from rclpy.node import Node
import rclpy


class clock_node(Node):

    def __init__(self):
        super().__init__('clock_node')

        # Publishes the latest clock at 20 Hz
        clock_timer = self.create_timer(0.05, self.clockCb)
        self.clock_pub = self.create_publisher(Clock, '/clock', 1)

    def clockCb(self):
        current_time = self.get_clock().now().to_msg()
        clock_msg = Clock()
        clock_msg.clock = current_time
        self.clock_pub.publish(clock_msg)


def main(args=None):
    rclpy.init(args=args)
    clock = clock_node()
    rclpy.spin(clock)
    clock_node.destroy_node()
    rclpy.shutdown()