'''
Package: map_management
   Filemapping.map_management.map_management.map as mapp_management_node.py
 Author: Will Heitman (w at heit dot mn)

Node to subscribe to, provide, and handle map data, routes,
and related functions.
'''

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

import pymap3d as pm

from xml.etree import ElementTree as ET
from map_management.map import Map


class MapManagementNode(Node):

    def __init__(self):
        super().__init__('map_management')
        self.map_string_sub = self.create_subscription(
            String, '/carla/map', self.map_string_cb, 10
        )

        self.gnss_sub = self.create_subscription(
            NavSatFix, '/carla/hero/gnss', self.gnss_cb, 10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odometry', 10
        )

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )

        self.get_logger().info("Waiting for map data...")
        self.map = None
        self.clock = Clock()

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def map_string_cb(self, msg: String):
        self.get_logger().info("Received map. Processing now...")
        map_string: str = msg.data

        self.map = Map(map_string)

        self.get_logger().info("{},{}".format(self.map.lat0, self.map.lon0))

    def gnss_cb(self, msg: NavSatFix):
        if self.map is None:
            return
        enu_xyz = pm.geodetic2enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self.map.lat0,
            self.map.lon0,
            0.0
        )
        self.get_logger().info("{},{},{}".format(enu_xyz[0], enu_xyz[1], enu_xyz[2]))

        odom_msg = Odometry()

        # The odometry is for the current time-- right now
        odom_msg.header.stamp = self.clock.clock

        # The odometry is the car's location on the map,
        # so the child frame is "base_link"
        odom_msg.header.frame_id = '/map'
        odom_msg.child_frame_id = '/base_link'

        pos = Point()
        pos.x = enu_xyz[0]
        pos.y = enu_xyz[1]
        pos.z = enu_xyz[2]
        odom_msg.pose.pose.position = pos
        # TODO: Orientation and covariance. WSH.

        # Publish our odometry message, converted from GNSS
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)

    lidar_processor = MapManagementNode()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
