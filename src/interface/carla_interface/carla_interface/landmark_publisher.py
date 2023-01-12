'''
Package:   carla
Filename:  landmark_publisher.py
Author:    Will Heitman (w at heit.mn)

Using depth camera and semantic camera data, publish an array of landmarks
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from carla_msgs.msg import CarlaRoute
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class LeaderboardLiaisonNode(Node):

    def __init__(self):
        # self.get_logger().info("woo")
        super().__init__('liaison_node')
        self.status_pub = self.create_publisher(
            Bool, '/carla/hero/status', qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        status_timer = self.create_timer(1.0, self.publish_hero_status)

        self.waypoint_sub = self.create_subscription(
            CarlaRoute, '/carla/hero/global_plan', self.route_cb, 10)
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        self.wp_marker_pub = self.create_publisher(
            MarkerArray, '/viz/wayppoints', 10)

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def route_cb(self, msg: CarlaRoute):
        self.get_logger().info("Received route with {} waypoints".format(len(msg.poses)))
        self.route = msg

        wp_marker_array = MarkerArray()
        wp_marker = Marker()
        wp_marker.header.frame_id = 'map'
        wp_marker.header.stamp = self.clock.clock
        wp_marker.frame_locked = True
        wp_marker.ns = 'waypoints'
        wp_marker.type = Marker.ARROW
        wp_marker.action = Marker.ADD
        marker_color = ColorRGBA()
        marker_color.r, marker_color.g, marker_color.b, marker_color.a = (
            1.0, 1.0, 1.0, 10)

        wp_marker.color = marker_color
        wp_marker.scale.x = 5.0
        wp_marker.scale.y = 2.0
        wp_marker.scale.z = 2.0

        for pose in msg.poses:
            wp_marker.pose = pose
            wp_marker_array.markers.append(wp_marker)

        self.wp_marker_pub.publish(wp_marker_array)

    def publish_hero_status(self):
        """Publish a true Bool to the leaderboard status topic"""
        status = Bool()
        status.data = True  # This means "good to go!"
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)

    liaison_node = LeaderboardLiaisonNode()

    rclpy.spin(liaison_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    liaison_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
