# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from voltron_msgs.msg import CostedPaths, CostedPath, ZoneArray, Zone
import math

class VizSubscriber(Node):

    def __init__(self):
        super().__init__('viz_subscriber')
        self.costed_paths_sub = self.create_subscription(CostedPaths, '/planning/paths', self.paths_cb, 10)
        self.zone_sub = self.create_subscription(ZoneArray, '/planning/zone_array', self.zone_cb, 10)
        self.path_viz_pub = self.create_publisher(Marker, '/viz/path', 10)
        self.zone_viz_pub = self.create_publisher(Marker, '/viz/zone', 10)

    def paths_cb(self, msg: CostedPaths):
        # self.get_logger().info("Received {} paths".format(len(msg.paths)))
        line_strip = Marker()
        line_strip.header.frame_id = 'map'
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.action = Marker.MODIFY
        line_strip.type = Marker.LINE_STRIP
        line_strip.ns = 'paths'
        line_strip.color = ColorRGBA(
            r = 0.0,
            g = 1.0,
            b = 0.0,
            a = 1.0
        )
        line_strip.pose.position.z = 0.6 # Offset from road surface
        line_strip.scale.x = 0.5 # Meters wide.
        line_strip.frame_locked = True
        line_strip.points = msg.paths[0].points
        self.path_viz_pub.publish(line_strip)
    
    def zone_cb(self, msg: ZoneArray):
        self.get_logger().info("Received {} zones".format(len(msg.zones)))
        if len(msg.zones) == 0:
            return
        line_strip = Marker()
        line_strip.header.frame_id = 'map'
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.action = Marker.MODIFY
        line_strip.type = Marker.LINE_STRIP
        line_strip.ns = 'zones'
        line_strip.color = ColorRGBA(
            r = 0.0,
            g = 1.0,
            b = 0.0,
            a = 1.0
        )
        line_strip.pose.position.z = 0.6 # Offset from road surface
        line_strip.scale.x = 0.5 # Meters wide.
        line_strip.scale.y = 0.5
        line_strip.scale.z = 0.5
        line_strip.points = msg.zones[0].poly.points
        self.zone_viz_pub.publish(line_strip)

def main(args=None):
    rclpy.init(args=args)

    viz_subscriber = VizSubscriber()

    rclpy.spin(viz_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
