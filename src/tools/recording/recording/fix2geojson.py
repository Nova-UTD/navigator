"""
Package:   recording
Filename:  fix2geojson.py
Author:    Will Heitman (w at heit.mn)

Subscribes to NavSatFix, writes to .geojson

"""

import math
import sys  # argv
from array import array as Array
from datetime import datetime
from time import sleep, strftime, strptime, time

# from shapely import Point, LineString
# import shapely

import numpy as np
import rclpy

# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from navigator_msgs.msg import VehicleSpeed


class FixToGeoJSONNode(Node):
    def __init__(self):
        super().__init__("FixToGeoJSONNode")

        self.last_position = []
        # fix_sub = self.create_subscription(NavSatFix, "/gnss/fix", self.fixCb, 1)
        odom_sub = self.create_subscription(Odometry, "/gnss_gt/odometry", self.odomCb, 1)

        self.speed = 0
        speed_sub = self.create_subscription(VehicleSpeed, "/speed", self.speedCb, 1)
        
        self.coordinates = []
        self.record_position_timer = self.create_timer(0.2, self.record_position, callback_group=MutuallyExclusiveCallbackGroup())

    def speedCb(self, msg: VehicleSpeed):
        self.speed = msg.speed

    def close(self):
        # ls = LineString(self.coordinates)

        # json = shapely.to_geojson(ls)

        # with open("fix.geojson", "w") as f:
        #     f.write(json)

        with open("trace.csv", "w") as f:
            for coord in self.coordinates:
                f.write(f"{coord[0]},{coord[1]}\n")

        self.get_logger().info(f"Wrote {len(self.coordinates)} points")

    def odomCb(self, msg: Odometry):
        self.last_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def record_position(self):
        if len(self.last_position) == 0:
            return
        # record new waypoint if it is farther than 1 meter from the last point.
        if len(self.coordinates) == 0 or math.sqrt( (self.coordinates[-1][0]-self.last_position[0])**2 + (self.coordinates[-1][1]-self.last_position[1])**2 ) > 1.0:
            self.coordinates.append(self.last_position)

    # def fixCb(self, msg: NavSatFix):
    #     coord = [msg.longitude, msg.latitude]
    #     self.coordinates.append(coord)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FixToGeoJSONNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         # Write any remaining data to the file before closing.
#         node.close()
#     FixToGeoJSONNode.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FixToGeoJSONNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Write any remaining data to the file before closing.
        node.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()