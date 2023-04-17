"""
Package:   recording
Filename:  fix2geojson.py
Author:    Will Heitman (w at heit.mn)

Subscribes to NavSatFix, writes to .geojson

"""

import os
import sys  # argv
from array import array as Array
from datetime import datetime
from time import sleep, strftime, strptime, time

from shapely import Point, LineString
import shapely

import numpy as np
import rclpy

# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


class FixToGeoJSONNode(Node):
    def __init__(self):
        super().__init__("FixToGeoJSONNode")

        fix_sub = self.create_subscription(NavSatFix, "/gnss/fix", self.fixCb, 1)
        self.coordinates = []

    def close(self):
        ls = LineString(self.coordinates)

        json = shapely.to_geojson(ls)

        with open("fix.geojson", "w") as f:
            f.write(json)

        self.get_logger().info(f"Wrote {len(self.coordinates)}")

    def fixCb(self, msg: NavSatFix):
        coord = Point(msg.longitude, msg.latitude)
        self.coordinates.append(coord)


def main(args=None):
    rclpy.init(args=args)
    node = FixToGeoJSONNode()
    try:
        rclpy.spin(node)
    finally:
        # Write any remaining data to the file before closing.
        node.close()
    FixToGeoJSONNode.destroy_node()
    rclpy.shutdown()
