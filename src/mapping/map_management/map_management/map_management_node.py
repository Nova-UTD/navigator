'''
Package: map_management
   File: map_management_node.py
 Author: Will Heitman (w at heit dot mn)

Node to subscribe to, provide, and handle map data, routes,
and related functions.
'''

import math
from array import array as Array
from xml.etree import ElementTree as ET

import numpy as np
import opendrivepy as odrpy
import pymap3d as pm
import rclpy
import ros2_numpy as rnp
from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from opendrivepy.map import Map
from rclpy.node import Node, QoSProfile
from rclpy.qos import DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker


class MapManagementNode(Node):

    def __init__(self):
        super().__init__('map_management')
        self.odr_map: Map = None

        self.map_string_sub = self.create_subscription(
            String, '/carla/map', self.map_string_cb, 10
        )

        self.gnss_sub = self.create_subscription(
            NavSatFix, '/carla/hero/gnss', self.gnss_cb, 10
        )

        self.world_info_sub = self.create_subscription(
            CarlaWorldInfo, '/carla/world_info', self.world_info_cb, 10
        )

        self.world_info_pub = self.create_publisher(
            CarlaWorldInfo, '/carla/world_info', 10
        )

        self.world_info_repub_timer = self.create_timer(
            0.5, self.repub_world_info)

        self.odom_pub = self.create_publisher(
            Odometry, '/odometry/gnss', 10
        )

        self.odom_raw_pub = self.create_publisher(
            Odometry, '/odometry_raw', 10
        )

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )

        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/map', qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        self.grid_bounds_pub = self.create_publisher(
            Marker, '/grid/bounds', 10)

        self.map_string = ""

        self.grid_pub_timer = self.create_timer(1, self.publish_map_grid)

        self.get_logger().info("Waiting for map data...")
        self.map = None
        self.clock = Clock()
        self.world_info = CarlaWorldInfo()

    def publish_map_grid(self):
        if (self.road_grid is None):
            return
        self.grid_pub.publish(self.road_grid)

        marker = Marker()
        marker.action = Marker.ADD
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.frame_locked = True
        marker.header.frame_id = 'map'
        marker.header.stamp = self.clock.clock
        marker.id = 1
        marker.ns = 'grid'
        marker.pose.position.x = self.odr_map.header.x_0
        marker.pose.position.y = self.odr_map.header.y_0
        marker.scale.x = 20.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0
        marker.type = Marker.ARROW

        self.grid_bounds_pub.publish(marker)

    def clock_cb(self, msg: Clock):
        if self.map_string == "":
            with open('/navigator/map_string.xml', 'r') as f:
                self.map_string = f.read()
            map_msg = String()
            map_msg.data = self.map_string
            self.clock = msg
            self.map_string_cb(map_msg)

    def map_string_cb(self, msg: String):
        self.get_logger().info("Received map. Processing now...")
        map_string: str = msg.data

        grid_resolution = 0.4
        self.odr_map: Map = Map(map_string, grid_resolution=grid_resolution)

        road_grid_arr: np.array = self.odr_map.road_grid
        road_grid_msg = OccupancyGrid()
        road_grid_msg.data = Array('b', road_grid_arr.ravel().astype(np.int8))

        meta = MapMetaData()
        meta.height = road_grid_arr.shape[0]
        meta.width = road_grid_arr.shape[1]
        meta.map_load_time = self.clock.clock
        meta.resolution = grid_resolution

        origin = Pose()
        origin.position.x = self.odr_map.header.west_bound
        origin.position.y = self.odr_map.header.south_bound
        # origin.orientation.w = math.cos(-1*math.pi / 4)
        # origin.orientation.z = math.sin(-1*math.pi / 4)
        meta.origin = origin

        print(
            f"Grid origin: ({origin.position.x},{origin.position.y},{origin.position.z}) @ {math.acos(origin.orientation.z)*2}")

        road_grid_msg.info = meta
        road_grid_msg.header.frame_id = 'map'
        road_grid_msg.header.stamp = self.clock.clock
        self.road_grid = road_grid_msg

    def gnss_cb(self, msg: NavSatFix):
        if self.odr_map is None:
            return
        enu_xyz = pm.geodetic2enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self.odr_map.header.lat_0,
            self.odr_map.header.lon_0,
            0.0
        )
        odom_msg = Odometry()

        # The odometry is for the current time-- right now
        odom_msg.header.stamp = self.clock.clock

        # The odometry is the car's location on the map,
        # so the child frame is "base_link"
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        pos = Point()

        pos.x = enu_xyz[0]
        pos.y = enu_xyz[1]
        pos.z = enu_xyz[2]
        odom_msg.pose.pose.position = pos
        self.odom_pub.publish(odom_msg)

    def world_info_cb(self, msg: CarlaWorldInfo):
        self.world_info = msg

    def repub_world_info(self):
        self.world_info_pub.publish(self.world_info)


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
