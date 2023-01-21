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
from geometry_msgs.msg import PoseStamped, Polygon, Point32, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nova_msgs.msg import CostedPaths, CostedPath, Zone, ZoneArray, Trajectory
import math

class NovaVizNode(Node):

    def __init__(self):
        super().__init__('nova_viz_node')
        self.get_logger().info("woo")
        self.trajectory_sub = self.create_subscription(Trajectory, '/planning/outgoing_trajectory', self.motion_paths_cb, 10)

        self.zones_viz_pub = self.create_publisher(MarkerArray, '/viz/zones', 10)
        self.trajectory_viz_pub = self.create_publisher(MarkerArray, '/viz/trajectory', 10)
        self.zones_sub = self.create_subscription(ZoneArray, '/planning/zone_array', self.zones_cb, 10)

    def zones_cb(self, msg: ZoneArray):
        self.get_logger().info("Received {} zones".format(len(msg.zones)))
        marker_array = MarkerArray()
        for idx, zone in enumerate(msg.zones):
            zone_marker = Marker()
            zone_marker.header = msg.header
            zone_marker.id = idx
            zone_marker.ns = 'zones'
            zone_marker.frame_locked = True
            zone_marker.type = zone_marker.LINE_STRIP
            zone_marker.scale.x = 1.0
            zone_marker.scale.y = 1.0

            zone_color = ColorRGBA()
            zone_color.a = 1.0
            zone_color.r = 1.0
            zone_color.g = 0.0
            zone_color.b = 0.0
            zone_marker.color = zone_color

            points = []
            for point32 in zone.poly.points:
                self.get_logger().info(str(point32))
                pt = Point()
                pt.x = point32.x
                pt.y = point32.y
                pt.z = point32.z
                points.append(pt)

            self.get_logger().info(str(points))
            zone_marker.points = points
            zone_marker.points.append(points[0]) # Complete the loop
            marker_array.markers.append(zone_marker)
        
        self.zones_viz_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    viz_subscriber = NovaVizNode()

    rclpy.spin(viz_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
