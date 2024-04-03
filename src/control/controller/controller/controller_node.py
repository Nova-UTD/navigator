#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2023 - https://github.com/Nova-UTD/navigator/
# Daniel Vayman - Daniel.Vayman@utdallas.edu

from typing import List
from navigator_msgs.msg import Trajectory, TrajectoryPoint, PeddlePosition, SteeringPosition
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import collections

class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')

        


def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()