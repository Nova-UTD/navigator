'''
Package:   airbags
Filename:  airbag_node.py
Author:    Will Heitman (w at heit.mn)

Code to establish safety zones around the car where the speed is limited.
'''

from matplotlib import pyplot as plt
from navigator_msgs.msg import VehicleControl
from navigator_msgs.msg import CarlaSpeedometer, VehicleControl
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import ros2_numpy as rnp
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from matplotlib.patches import Rectangle


class AirbagNode(Node):
    def __init__(self):
        super().__init__('airbag_node')

        self.speed_limit = 0.  # m/s
        self.current_speed = 0.  # m/s

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/filtered', self.lidarCb, 1)

        # Don't konw what this topic is
        #self.command_sub = self.create_subscription(
        #   VehicleControl, '/control/unlimited', self.commandCb, 1)

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedCb, 1)

        # Publishes corrected, limited commands
        self.command_pub = self.create_publisher(
            VehicleControl, '/vehicle/control', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/status/airbags', 1)

        self.marker_pub = self.create_publisher(
            Marker, '/visuals/airbags', 1)

        self.current_airbag_pub = self.create_publisher(
            String, '/planning/current_airbag', 1)

        # Clock subscription
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)
        self._cached_clock_ = Clock()

    def speedCb(self, msg: CarlaSpeedometer):
        self.current_speed = msg.speed

    def distanceToSpeedLimit(self, dist: float):
        """Map distance to max speed. This should be very conservative,
        limiting speed as little as possible. This means that while safety
        is considered, comfort (jerky motion) is not. The motion planner
        would ideally never hit this speed limit outside of an emergency.

        Args:
            dist (float): Distance from closest point in front of the car (m)

        Returns:
            float: speed limit (m/s)
        """

        # x = 2y+2 produces:
        # 0 m/s at <1 meter
        # 10 m/s (~23mph) at 21 meters

        ZERO_POINT = 2  # meters. Distances at or below will cause car to stop

        return max((dist-2)/2, 0)

    def lidarCb(self, msg: PointCloud2):
        data = rnp.numpify(msg)

        # We only consider points in front of the car, not too far
        # off to the side. Assume our car is ~2 meters wide.
        # Filter out all other points.
        data = data[data['y'] > -1.1]
        data = data[data['y'] < 1.1]
        data = data[data['x'] >= 0.0]

        # Of the remaining points, find the minimum x value.
        if len(data) == 0:
            closest_x = 999.9
        else:
            closest_x = np.min(data['x'])

        self.speed_limit = self.distanceToSpeedLimit(closest_x)

    def commandCb(self, msg: VehicleControl):

        if self.current_speed > self.speed_limit:
            speed_over_limit = self.current_speed - self.speed_limit
            self.get_logger().warning(
                f"Speed is {speed_over_limit} m/s over limit!")
            BRAKING_FORCE = 0.5
            msg.throttle = 0.0
            msg.brake = speed_over_limit * BRAKING_FORCE

        self.command_pub.publish(msg)

    def clockCb(self, msg: Clock):
        self._cached_clock_ = msg


def main(args=None):
    rclpy.init(args=args)

    airbag_node = AirbagNode()

    rclpy.spin(airbag_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airbag_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
