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
  '''
  Publishes localized pose, using rough gps pose for an initial pose estimate 
  and kiss-icp for odometry
  '''
  def __init__(self):
    super().__init__("localization_node")
    # kiss-icp pipeline setup with pre-made PCD map
    self.kiss_config = KISSConfig()
    self.kiss_config.mapping.voxel_size = VOXEL_SIZE
    self.odometry = KissICP(self.kiss_config, PCD)

    # subscribe to lidar and rough gps pos, publish localized pose.
    self.pcdSub = self.create_subscription(PointCloud2, '/lidar/filtered', self.register, 1)
    self.pcdSub = self.create_subscription(Odometry, '/gnss/odometry', self.initialPos, 1)
    self.stampPosePub = self.create_publisher(Odometry, '/localized_pose', 1)
    
    # Variable inits
    self.first = True
    self.initial_pos_gathered = False
    self.get_logger().info("localization node init")
    self.gpsPoses = np.zeros((3,3))
    self.gpsCount = 0
    self.initial_pos = np.eye(4)

  def initialPos(self, gnss):
      # Only gather gps pos if we haven't yet successfully performed our initial registration
      if self.first:
        self.gpsPoses[self.gpsCount][0] = gnss.pose.pose.position.x
        self.gpsPoses[self.gpsCount][1] = gnss.pose.pose.position.y
        self.gpsPoses[self.gpsCount][2] = gnss.pose.pose.position.z
        self.gpsCount += 1

        # Collect average of 3 gps readings.
        if self.gpsCount == 3:
          finalGPSPose = np.mean(self.gpsPoses, axis=0)
          self.initial_pos[0][3] = finalGPSPose[0]
          self.initial_pos[1][3] = finalGPSPose[1]
          self.initial_pos[2][3] = finalGPSPose[2]
          self.initial_pos_gathered = True
          self.gpsCount = 0
          self.get_logger().info("Determined average GPS pos")

  def register(self, pcd):
    if not self.initial_pos_gathered: return
    pcd = rnp.numpify(pcd, PointCloud2)
    num_points = pcd.shape[0]
    pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T

    # global registration for initial pose
    if self.first:
      self.get_logger().info("globally registering")
      
      # Preprocessing
      target = o3d.io.read_point_cloud(PCD)
      target = target.voxel_down_sample(VOXEL_SIZE)
      o3d_pcd = o3d.geometry.PointCloud()
      o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
      o3d_pcd = o3d_pcd.remove_non_finite_points(remove_nan=True, remove_infinite=True)
      o3d_pcd = o3d_pcd.voxel_down_sample(VOXEL_SIZE)
      
      # Global Map Cropping based on gps data
      local_extent = o3d_pcd.get_axis_aligned_bounding_box().get_extent()
      buffer = np.array([20.0, 20.0, 20.0])
      crop_extent = local_extent + buffer * 2
      scan_center = self.initial_pos[:3, 3]
      min_bound = scan_center - crop_extent / 2
      max_bound = scan_center + crop_extent / 2
      aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
      target_crop = target.crop(aabb)

      # Feature generation
      radius_normal = VOXEL_SIZE * 2
      o3d_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
      radius_feature = VOXEL_SIZE * 5
      pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        o3d_pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
      target_crop.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
      #gurting??
      target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_crop,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
      
      # Registration
      distance_threshold = VOXEL_SIZE
      result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        o3d_pcd, target_crop, pcd_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.8))
      
      # Sanity check of fitness, throw out horrible matches
      if result.fitness < 0.5: return
      self.get_logger().info(str(result.transformation[0, 3]) + ", " + str(result.transformation[1, 3]))
      self.odometry.last_pose = result.transformation
      self.first = False
      
    # kiss-icp odometry stepping
    timestamps = np.linspace(0, 1, num=num_points, dtype=np.float32)

    self.odometry.register_frame(pcd, timestamps)

    current_pose = self.odometry.last_pose

    # Outputting localized pose
    pub_msg = Odometry()
    pub_msg.header.stamp = self.get_clock().now().to_msg()
    pub_msg.header.frame_id = "map"
    pub_msg.child_frame_id = "lidarCar"

    # populate position
    pub_msg.pose.pose.position.x = current_pose[0, 3]
    pub_msg.pose.pose.position.y = current_pose[1, 3]
    pub_msg.pose.pose.position.z = current_pose[2, 3]

    self.get_logger().info("X: " + str(pub_msg.pose.pose.position.x) + "\n" + 
                           "Y: " + str(pub_msg.pose.pose.position.y) + "\n" + 
                           "Z: " + str(pub_msg.pose.pose.position.z) + "\n")

    q = quaternion_from_matrix(current_pose)

    # populate orientation
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