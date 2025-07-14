#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


import numpy as np
import os
from tf_transformations import quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory
import open3d as o3d

import ros2_numpy as rnp
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

#Path to Global PCD map
PCD = str(os.path.join(get_package_share_directory('lidar_SLAM'),
                   'resource', 'combined_map.pcd'))

INIT = str(os.path.join(get_package_share_directory('lidar_SLAM'),
                   'resource', 'init.txt'))

VOXEL_SIZE = 1

class LocalizationNode(Node):
  def __init__(self):
    super().__init__("localization_node")
    # kiss-icp pipeline setup with pre-made PCD map
    self.kiss_config = KISSConfig()
    self.kiss_config.mapping.voxel_size = VOXEL_SIZE
    self.odometry = KissICP(self.kiss_config, PCD)
    self.pcdSub = self.create_subscription(PointCloud2, '/lidar/filtered', self.register, 1)
    self.stampPosePub = self.create_publisher(Odometry, '/localized_pose', 1)
    self.first = True

  def register(self, pcd):
    pcd = rnp.numpify(pcd, PointCloud2)
    num_points = pcd.shape[0]
    pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T

    # global registration for initial pose
    if self.first:
      target = o3d.io.read_point_cloud(PCD)
      o3d_pcd = o3d.geometry.PointCloud()
      o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
      o3d_pcd = o3d_pcd.remove_non_finite_points(remove_nan=True, remove_infinite=True)
      o3d_pcd = o3d_pcd.voxel_down_sample(VOXEL_SIZE)
      radius_normal = VOXEL_SIZE * 2
      o3d_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
      radius_feature = VOXEL_SIZE * 5
      pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        o3d_pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
      target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
      target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
      
      distance_threshold = VOXEL_SIZE
      result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        o3d_pcd, target, pcd_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.8))
      if result.fitness < 0.5: return
      self.get_logger().info(str(result.transformation[0, 3]) + ", " + str(result.transformation[1, 3]))
      self.odometry.last_pose = result.transformation
      self.first = False
      
    # kiss-icp odometry stepping
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