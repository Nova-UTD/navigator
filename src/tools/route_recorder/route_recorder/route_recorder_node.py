'''
Package:   helpers
Filename:  routing_monitor.py

Subscribes to:
/planning/rough_route (Path, published by an interface - CARLA, Real World)
/tf

Calls:
/set_route (navigator_msgs/srv/SetRoute, served by MapManager)
'''

import math

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from navigator_msgs.srv import SetRoute
from nav_msgs.msg import Path
from rclpy.node import Node
import rclpy
from nav_msgs.msg import Odometry
from shapely.geometry import LineString

class RouteRecorder(Node):

    def __init__(self):
        super().__init__('route_recorder_node')

        self.gnss_subscriber = self.create_subscription(Odometry, "/gnss/odometry", self.odom_callback, 1)
        self.odom_frames = []
        self.current_odom = None

        self.odom_timer = self.create_timer(1, self.odom_timer_callback)

    def odom_timer_callback(self):
        if not self.current_odom:
            return
        self.odom_frames.append((self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y))


    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def close(self):
        with open("/navigator/data/maps/random_ass_parking_lot.txt", "w") as file:
            self.get_logger().info(f"{self.odom_frames}")
            line_string = LineString(self.odom_frames)
            file.write(str(line_string))

    


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    rclpy.shutdown()