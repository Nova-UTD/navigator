'''
Package:   recording
Filename:  odom2tf.py
Author:    Will Heitman (w at heit.mn)

Loads and plays all recordings from a given directory,
publishing the data as ROS messages.

Arguments (optional):

'from=hh-mm': Start recording at this file, continuing until either
              'to=' or end.
    Ex: 'from=17-36' starts at 5:36pm today
        'from=04-04_17-36' starts at April 4, 5:36pm

'to=': Same format as 'from'.

'src=': Source directory for recordings. Default: '/navigator/recordings'
'''

import os
import sys  # argv
from array import array as Array
from datetime import datetime
from time import sleep, strftime, strptime, time

import numpy as np
import rclpy
# Message definitions
from navigator_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


class odom2tf(Node):

    def __init__(self):
        super().__init__('odom2tf')

        odom_sub = self.create_subscription(Odometry, '/gnss/odometry', self.odomCb, 1)
        self.speed_pub = self.create_publisher(CarlaSpeedometer, '/speed', 1)

        self.tf_broadcaster = TransformBroadcaster(self)

    def odomCb(self, msg: Odometry):

        speed_msg = CarlaSpeedometer()
        speed_msg.speed = msg.twist.twist.linear.x
        self.speed_pub.publish(speed_msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        print(f"Sent tf {t}")


def main(args=None):
    rclpy.init(args=args)
    node = odom2tf()
    rclpy.spin(node)
    odom2tf.destroy_node()
    rclpy.shutdown()
