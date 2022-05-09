import pygame
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, ColorRGBA, Float32
from geometry_msgs.msg import PoseStamped, Polygon, Point32, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from voltron_msgs.msg import CostedPaths, CostedPath, Zone, ZoneArray, Obstacle3DArray, Obstacle3D, Trajectory
import math


class NovaUXNode(Node):

    def __init__(self):
        super().__init__('nova_ux_node')

        self.get_logger().info("woo")
        self.desired_speed_sub = self.create_subscription(
            Float32, '/planning/outgoing_trajectory', self.motion_paths_cb, 10)

        


def main(args=None):
    rclpy.init(args=args)

    viz_subscriber = NovaUXNode()

    rclpy.spin(viz_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
