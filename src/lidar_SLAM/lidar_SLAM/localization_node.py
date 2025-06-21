#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from math import sqrt
import numpy as np
import os
from tf_transformations import quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory

import ros2_numpy as rnp
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

#Path to Global PCD map
PCD = str(os.path.join(get_package_share_directory('lidar_SLAM'),
                   'resource', 'combined_map.pcd'))

INIT = str(os.path.join(get_package_share_directory('lidar_SLAM'),
                   'resource', 'init.txt'))

class LocalizationNode(Node):
  def __init__(self):
    super().__init__("localization_node")
    self.kiss_config = KISSConfig()
    self.kiss_config.mapping.voxel_size = 1.0
    self.odometry = KissICP(self.kiss_config, PCD)
    self.odometry.last_pose = np.loadtxt(INIT)
    self.pcdSub = self.create_subscription(PointCloud2, '/lidar/filtered', self.register, 1)
    self.stampPosePub = self.create_publisher(Odometry, '/localized_pose', 1)
  
  def register(self, pcd):
    pcd = rnp.numpify(pcd, PointCloud2)
    num_points = pcd.shape[0]
    pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T
    timestamps = np.linspace(0, 1, num=num_points, dtype=np.float32)

    self.odometry.register_frame(pcd, timestamps)

    current_pose = self.odometry.last_pose

    pub_msg = Odometry()
    pub_msg.header.stamp = self.get_clock().now().to_msg()
    pub_msg.header.frame_id = "map"
    pub_msg.child_frame_id = "lidarCar"

    pub_msg.pose.pose.position.x = current_pose[0, 3]
    pub_msg.pose.pose.position.y = current_pose[1, 3]
    pub_msg.pose.pose.position.z = current_pose[2, 3]

    self.get_logger().info("X: " + str(pub_msg.pose.pose.position.x) + "\n" + 
                           "Y: " + str(pub_msg.pose.pose.position.y) + "\n" + 
                           "Z: " + str(pub_msg.pose.pose.position.z) + "\n")

    q = quaternion_from_matrix(current_pose)

    pub_msg.pose.pose.orientation.x = q[0]
    pub_msg.pose.pose.orientation.y = q[1]
    pub_msg.pose.pose.orientation.z = q[2]
    pub_msg.pose.pose.orientation.w = q[3]

    covariance_matrix = np.zeros(36, dtype=float)
    covariance_matrix[0] = 2.0  # Variance in x
    covariance_matrix[7] = 2.0  # Variance in y
    # All other diagonal elements for z, roll, pitch, yaw are 0.0
    # You could set them to a small number to indicate high certainty, e.g., 1e-9
    covariance_matrix[14] = 1e-9 # var(z)
    covariance_matrix[21] = 1e-9 # var(roll)
    covariance_matrix[28] = 1e-9 # var(pitch)
    covariance_matrix[35] = 1e-9 # var(yaw)
    
    pub_msg.pose.covariance = covariance_matrix.tolist()

    self.stampPosePub.publish(pub_msg)


def main(args=None):
  rclpy.init(args=args)

  node = LocalizationNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__=='__main__':
  main()