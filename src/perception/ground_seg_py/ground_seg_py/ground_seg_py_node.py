import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import sys
import time
import math
import pypatchworkpp
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from pygroundsegmentation import GroundPlaneFitting

import image_geometry
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2

import struct


class GroundSegNode(Node):

    def __init__(self):
        super().__init__('ground_seg_py_node')

        self.carla_clock = Clock()

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10
        )

        self.raw_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.pwpp, 10
        )

        self.filtered_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/filtered', 10
        )

        self.ground_lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/ground', 10
        )


    def clock_cb(self, msg: Clock):
        self.carla_clock = msg

    def pwpp(self, msg: PointCloud2):
        params = pypatchworkpp.Parameters()
        #params.verbose = False
        PatchworkPLUSPLUS = pypatchworkpp.patchworkpp(params)

        xyzi = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(xyzi)
        PatchworkPLUSPLUS.estimateGround(xyz)
        ground_ids = PatchworkPLUSPLUS.getNongroundIndices()
        #print(nonground)
        nonground = np.take(xyzi, ground_ids)
        f_msg = rnp.point_cloud2.array_to_pointcloud2(nonground, msg.header.stamp, msg.header.frame_id)
        self.filtered_lidar_pub.publish(f_msg)

    def point_cloud_cb(self, msg: PointCloud2):
        start0 = time.time()
        ground_estimator = GroundPlaneFitting(
            1,      # Divide evenly the point cloud into a number of segments
            3,      # Number of iterations
            250,    # number of points used to estimate the LPR
            10,    # Threshold for points to be considered initial seeds
            0.3,    # Threshold distance from the plane
            2.5,      # LiDAR sensor height to ground
            1,
        ) #Instantiate one of the Estimators

        cloud_range = 80.0
        max_height = 2.5  # Exclude points above this height, in meters

        start1 = time.time()
        xyzi = rnp.point_cloud2.pointcloud2_to_array(msg)
        #xyzi = np.array(list(map(list,xyzi)))
        #point_range_filter(xyzi, [-cloud_range, -cloud_range, -max_height, cloud_range, cloud_range, max_height])
        #xyzi = np.array(list(map(tuple,xyzi)))
        #temp = pc2.create_cloud(msg.header, msg.fields, xyzi)
        #xyzi = rnp.point_cloud2.pointcloud2_to_array(temp)
        xyz = rnp.point_cloud2.get_xyz_points(xyzi)
        end1 = time.time()
        length1 = start1 - end1
        start2 = time.time()

        ground_idxs = ground_estimator.estimate_ground(xyz)
        #ground_pcl = xyzi[ground_idxs]
        not_ground_pcl = np.delete(xyzi, ground_idxs)

        #fields = [PointField(name='x', offset=0, datatype=7, count=1), PointField(name='y', offset=4, datatype=7, count=1), PointField(name='z', offset=8, datatype=7, count=1), PointField(name='intensity', offset=16, datatype=7, count=1), PointField(name='t', offset=20, datatype=6, count=1), PointField(name='reflectivity', offset=24, datatype=4, count=1), PointField(name='ring', offset=26, datatype=4, count=1), PointField(name='ambient', offset=28, datatype=4, count=1), PointField(name='range', offset=32, datatype=6, count=1)]

        print(msg.header)
        print(msg.fields)

        #f_msg = pc2.create_cloud(msg.header, msg.fields, not_ground_pcl)
        f_msg = rnp.point_cloud2.array_to_pointcloud2(not_ground_pcl, msg.header.stamp, msg.header.frame_id)
        #g_msg = pc2.create_cloud(msg.header, msg.fields, ground_pcl)
        #f_msg.header = msg.header
        #print(3)
        end2 = time.time()
        length2 = start2 - end2
        self.filtered_lidar_pub.publish(f_msg)
        #self.ground_lidar_pub.publish(g_msg)
        #self.filtered_lidar_pub.publish(msg)
        end0 = time.time()
        length0 = start0 - end0
        print("V1 ", length0, length1, length2)

def point_range_filter(pts, point_range=[0, -39.68, -3, 69.12, 39.68, 1]):
    '''
    data_dict: dict(pts, gt_bboxes_3d, gt_labels, gt_names, difficulty)
    point_range: [x1, y1, z1, x2, y2, z2]
    '''
    flag_x_low = pts[:, 0] > point_range[0]
    flag_y_low = pts[:, 1] > point_range[1]
    flag_z_low = pts[:, 2] > point_range[2]
    flag_x_high = pts[:, 0] < point_range[3]
    flag_y_high = pts[:, 1] < point_range[4]
    flag_z_high = pts[:, 2] < point_range[5]
    flag_i = pts[:, 3] == True
    keep_mask = (flag_x_low & flag_y_low & flag_z_low & flag_x_high & flag_y_high & flag_z_high & flag_i)
    pts = pts[keep_mask]
    return pts

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
