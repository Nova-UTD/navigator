'''
Package:   parade_controller
Filename:  controller.py
Author:    Daniel Vayman

Controller for the hoco parade that follows our flag
'''

from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node


class ParadeController(Node):
    def __init__(self):
        super().__init__('parade_controller')
        

def main(args=None):
    rclpy.init(args=args)

    parade_controller_node = ParadeController()

    rclpy.spin(parade_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parade_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
