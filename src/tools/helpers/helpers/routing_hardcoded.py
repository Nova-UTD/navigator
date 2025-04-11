'''
Package:   helpers
Filename:  routing_hardcoded.py

This node fills a gap while we wait for the Map Manager to be updated.  This reads 
a csv of x-y points and publishes them as the /planning/route path

Subscribes to:
Nothing

Publishes:
/planning/route
'''

import math
import os.path

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from navigator_msgs.srv import SetRoute
from nav_msgs.msg import Path
from rclpy.node import Node
import rclpy
import rclpy.time
import rclpy.utilities
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rosgraph_msgs.msg import Clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class RoutingHardCoded(Node):

    def __init__(self):
        super().__init__('routing_hardcoded_node')

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        param_filename = self.declare_parameter('filename', '')
        filename = param_filename.get_parameter_value().string_value

        self.smooth_route_pub = self.create_publisher(Path, '/planning/route', 1, callback_group=MutuallyExclusiveCallbackGroup())

        smooth_route_timer = self.create_timer(1, self.smooth_route_pub_tick, callback_group=MutuallyExclusiveCallbackGroup())

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.clock = Clock().clock

        self.smooth_route_msg = Path()
        self.smooth_route_msg.header.stamp = self.clock
        self.smooth_route_msg.header.frame_id = 'map'
        self.smooth_route_msg.poses = []
        try:
            with open(filename, "r") as f:
                for line in f.readlines():
                    line = line.strip()
                    if len(line) > 0:
                        coord = line.split(',')
                        p = PoseStamped()
                        p.header.stamp = self.clock
                        p.header.frame_id = 'base_link'
                        p.pose.position.x = float(coord[0])
                        p.pose.position.y = float(coord[1])
                        self.smooth_route_msg.poses.append(p)
            self.get_logger().info("Loaded a hard coded route with %i points." % len(self.smooth_route_msg.poses))
        except:
            self.get_logger().error("Could not load route file named: %s" % filename)

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def smooth_route_pub_tick(self):
        if self.smooth_route_msg is not None:
            self.smooth_route_msg.header.stamp = self.clock
            for i in range(len(self.smooth_route_msg.poses)):
                self.smooth_route_msg.poses[i].header.stamp = self.clock
            self.smooth_route_pub.publish(self.smooth_route_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoutingHardCoded()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()