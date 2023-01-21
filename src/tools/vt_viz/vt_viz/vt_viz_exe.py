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
from rclpy.duration import Duration

from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nova_msgs.msg import CostedPaths, CostedPath, ZoneArray, Zone, Trajectory
import math

class VizSubscriber(Node):

    def __init__(self):
        super().__init__('viz_subscriber')
        #self.costed_paths_sub = self.create_subscription(CostedPaths, '/planning/paths', self.paths_cb, 10)
        #self.path_viz_pub = self.create_publisher(Marker, '/viz/path', 10)
        self.trajectory_sub = self.create_subscription(Trajectory, '/planning/outgoing_trajectory', self.motion_paths_cb, 10)
        self.zones_sub = self.create_subscription(ZoneArray, '/planning/zones', self.zones_cb, 10)
        self.zones_viz_pub = self.create_publisher(MarkerArray, '/viz/zones', 10)
        self.trajectory_viz_pub = self.create_publisher(Marker, '/viz/trajectory', 10)

    def motion_paths_cb(self, msg: Trajectory):
        self.get_logger().info("Received {} points".format(len(msg.points)))
        line_strip = Marker()
        line_strip.header.frame_id = 'map'
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.action = Marker.MODIFY
        line_strip.type = Marker.LINE_STRIP
        line_strip.ns = 'paths'
        line_strip.color = ColorRGBA(
            r = 0.5,
            g = 1.0,
            b = 0.7,
            a = 1.0
        )
        line_strip.pose.position.z = 0.6 # Offset from road surface
        line_strip.scale.x = 0.5 # Meters wide.
        line_strip.scale.y = 0.5
        line_strip.scale.z = 0.5
        line_strip.frame_locked = True
        for pt in msg.points:
            out_point = Point()
            out_point.x = pt.x
            out_point.y = pt.y
            out_point.z = pt.vx*2
            line_strip.points.append(out_point)
        self.trajectory_viz_pub.publish(line_strip)

    def zones_cb(self, msg: ZoneArray):
        
        marker_array = MarkerArray()
        
        for idx, zone in enumerate(msg.zones):
            zone_marker = Marker()
            zone_marker.header.frame_id = "map";
            zone_marker.id = idx
            zone_marker.ns = 'zones'
            zone_marker.frame_locked = True
            zone_marker.type = zone_marker.LINE_STRIP
            zone_marker.scale.x = 2.0
            zone_marker.scale.y = 2.0
            zone_marker.scale.z = 2.0
            zone_marker.lifetime = Duration(seconds=0.2).to_msg()

            zone_color = ColorRGBA()
            zone_color.a = 1.0
            zone_color.r = 1.0
            zone_color.g = 1.0-math.exp(-zone.max_speed/5);
            zone_color.b = 0.0
            zone_marker.color = zone_color

            points = []
            for point32 in zone.poly.points:
                pt = Point()
                pt.x = point32.x
                pt.y = point32.y
                pt.z = point32.z
                points.append(pt)
            zone_marker.points = points
            zone_marker.points.append(points[0]) # Complete the loop
            marker_array.markers.append(zone_marker)
        self.zones_viz_pub.publish(marker_array)

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
