#!/usr/bin/python

from nav_msgs.msg import Odometry  # For GPS
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_msgs
from tf2_ros import TransformException, TransformStamped
from rclpy.node import Node
import rclpy



class LandmarkLocalizerNode(Node):

    def __init__(self):
        super().__init__('landmark_localizer')
        self.get_logger().info("Hello, world!")

def main(args=None):
    rclpy.init(args=args)

    node = LandmarkLocalizerNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
