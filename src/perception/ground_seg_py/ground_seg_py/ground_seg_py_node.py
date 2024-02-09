import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import sys
import time
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

import image_geometry
import matplotlib.pyplot as plt

import struct


class GroundSegNode(Node):

    def __init__(self):
        super().__init__('ground_seg_py_node')

        self.carla_clock = Clock()

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )

        self.raw_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.point_cloud_cb, 10
        )

        self.filtered_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/filtered', 10
        )


    def clock_cb(self, msg: Clock):
        self.carla_clock = msg


    def point_cloud_cb(self, msg: PointCloud2):
        #self.filtered_lidar_pub.publish(msg)
        data = rnp.numpify(msg)
        #print(data)
        filtered_pcd = []
    
        lidar_height = 0.0
        cloud_range = 80.0
        sens = 0.2
        res = 0.4  # Grid cell size, in meters
        max_height = 2.5  # Exclude points above this height, in meters

        grid_size = int(2 * math.ceil((cloud_range) / res) + 1)
        grid = grid_size * [grid_size * [[]]]

        center_x = int(math.ceil(cloud_range / res))
        center_y = int(math.ceil(cloud_range / res))

        for i in range(len(data)):
            x = data[i][0]
            y = data[i][1]
            z = data[i][2]

            if (abs(x) <= cloud_range) and (abs(y) <= cloud_range) and (z <= max_height):
                grid[int(center_x + round(x / res))][int(center_y + round(y / res))].append(i)

        #print(grid)

        hG = grid_size * [grid_size * [None]]
        hG[center_x][center_y] = -1 * lidar_height

        gridSeg = grid_size * [grid_size * [0]]
        gridSeg[center_x][center_y] = 1

        outerIndex = [None, None]

        for i in range(1, int(math.ceil(cloud_range / res)) + 1):
            outerIndex[0] = -1 * i
            outerIndex[1] = i

            for index in outerIndex:
                for k in range(-1 * i, i + 1):
                    currentCircle = {(center_x + index, center_y + k)}

                    if not ((k == -1 * i) or (k == i)):
                        currentCircle.add((center_x + k, center_y + index))

                    for pair in currentCircle:
                        x = pair[0]
                        y = pair[1]

                        H = float('-inf')
                        h = float('inf')

                        if grid[x][y]:
                            pcIndices = grid[x][y]

                            for j in range(len(pcIndices)):
                                H = max(data[pcIndices[j]][2], H)
                                h = min(data[pcIndices[j]][2], h)

                        hHatG = float('-inf')
                        innerCircleIndex = i - 1

                        xRelativeIndex = x - center_x
                        yRelativeIndex = y - center_y

                        for m in range(-1, 2):
                            for n in range(-1, 2):
                                xRelativeNew = abs(xRelativeIndex + m)
                                yRelativeNew = abs(yRelativeIndex + n)

                                if ((xRelativeNew == innerCircleIndex) and (yRelativeNew <= innerCircleIndex)):
                                    hGTemp = hG[x + m][y + n]
                                    hHatG = max(hGTemp, hHatG)

                        if (H != float('-inf')) and (h != float('inf')) and ((H - h) < sens) and ((H - hHatG) < sens):
                            gridSeg[x][y] = 1
                            hG[x][y] = H
                        else:
                            hG[x][y] = hHatG

                            if grid[x][y]:
                                pcIndices = grid[x][y]

                                for j in range(len(grid[x][y])):
                                    filtered_pcd.append(data[pcIndices[j]])

        #f_msg = rnp.msgify(PointCloud2, filtered_pcd)
        self.filtered_lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ground_seg_node = GroundSegNode()

    rclpy.spin(ground_seg_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ground_seg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
