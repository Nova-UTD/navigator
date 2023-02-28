'''
Package:   airbags
Filename:  airbag_node.py
Author:    Will Heitman (w at heit.mn)

Code to establish safety zones around the car where the speed is limited.
'''

from matplotlib import pyplot as plt
from carla_msgs.msg import CarlaEgoVehicleControl
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import ros2_numpy as rnp
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker

from matplotlib.patches import Rectangle


class Airbag:
    RED = 1
    AMBER = 2
    YELLOW = 3
    NONE = 4


# Extents in meters
RED_EXTENT = (1.0, 1.25)  # 1 meter in front, 1.25 on either side
AMBER_EXTENT = (3.0, 1.25)
YELLOW_EXTENT = (5.0, 1.75)


class AirbagNode(Node):
    def __init__(self):
        super().__init__('airbag_node')

        self.lidar_received_time = 0.0
        self.status = DiagnosticStatus()
        self.current_zone = Airbag.RED

        # Cost map subscription
        self.cost_sub = self.create_subscription(
            OccupancyGrid,
            '/grid/occupancy/current', self.costCb,
            10
        )

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

    def getSliceFromExtent(self, extent, res: float, origin_idx):
        extent_indices = np.ceil(
            np.asarray(extent) / res)
        extent_to = np.copy(origin_idx)
        extent_to[0] += extent_indices[0]
        extent_to[1] += extent_indices[1]

        extent_from = np.copy(origin_idx)
        extent_from[1] -= extent_indices[1]

        slice = np.s_[extent_from[1]:extent_to[1], extent_from[0]:extent_to[0]]

        return slice

    def costCb(self, msg: OccupancyGrid):
        # Convert to np array
        data = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        res = msg.info.resolution

        # Check if a point falls within each zone
        origin_cell = np.asarray([msg.info.origin.position.x,
                                  msg.info.origin.position.y]) * -1 / msg.info.resolution
        origin_cell = np.rint(origin_cell).astype(np.int8)  # Round to int
        plt.imshow(data, origin='lower')
        red_cells = data[self.getSliceFromExtent(RED_EXTENT, res, origin_cell)]

        amber_cells = data[self.getSliceFromExtent(
            AMBER_EXTENT, res, origin_cell)]
        yellow_cells = data[self.getSliceFromExtent(
            YELLOW_EXTENT, res, origin_cell)]

        marker = Marker()
        marker.type = Marker.CUBE
        marker.ns = 'airbags'
        marker.header = msg.header
        marker.frame_locked = True

        status_string = String()

        if np.sum(red_cells) > 0:
            self.current_zone = Airbag.RED
            print(f"Current zone: RED")
            marker.scale.x = RED_EXTENT[0] / 2
            marker.scale.y = RED_EXTENT[1]
            marker.scale.z = 0.1
            marker.pose.position.x = RED_EXTENT[0] / 2
            marker.color.r = 1.0
            marker.color.a = 1.0
            status_string.data = "RED"
        elif np.sum(amber_cells) > 0:
            self.current_zone = Airbag.AMBER
            print(f"Current zone: AMBER")
            marker.scale.x = AMBER_EXTENT[0] / 2
            marker.scale.y = AMBER_EXTENT[1]
            marker.scale.z = 0.1
            marker.pose.position.x = AMBER_EXTENT[0] / 2
            marker.color.r = 0.8
            marker.color.g = 0.3
            marker.color.a = 1.0
            status_string.data = "AMBER"

        elif np.sum(yellow_cells) > 0:
            self.current_zone = Airbag.YELLOW
            print(f"Current zone: YELLOW")
            marker.scale.x = YELLOW_EXTENT[0] / 2
            marker.scale.y = RED_EXTENT[1]
            marker.scale.z = 0.1
            marker.pose.position.x = RED_EXTENT[0] / 2
            marker.color.r = 0.5
            marker.color.g = 0.5
            status_string.data = "YELLOW"

        else:
            self.current_zone = Airbag.NONE
            print(f"Current zone: NONE")
            status_string.data = "NONE"

        self.marker_pub.publish(marker)
        self.current_airbag_pub.publish(status_string)

        # print(origin_cell)
        # plt.plot(origin_cell[0], origin_cell[1])
        # print(self.getSliceFromExtent(
        #     YELLOW_EXTENT, res, origin_cell))
        # print(yellow_cells)

        # plt.show()

    def commandCb(self, msg: CarlaEgoVehicleControl):
        # Check if a point
        return

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
