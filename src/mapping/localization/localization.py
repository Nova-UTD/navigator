# utility
from scipy.spatial.transform import Rotation
import numpy as np
import glob
import matplotlib.pyplot as plt
from numpy.lib.recfunctions import unstructured_to_structured
from scipy.spatial.transform import Rotation
import os
import open3d as o3d
import sys

# odometry + loop closure
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.voxelization import voxel_down_sample
from map_closures.map_closures import MapClosures
from map_closures.config import MapClosuresConfig

# ros
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
sys.path.append('/navigator/src/tools/ros2_numpy')
import ros2_numpy as rnp
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def transform_points(pcd, T):
    R = T[:3, :3]
    t = T[:3, -1]
    return pcd @ R.T + t


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.map_publisher = self.create_publisher(PointCloud2, '/local_map', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/vehicle_odom', 10)
        

        self.lidar_sub = self.create_subscription(
            PointCloud2, 
            '/lidar',
            self.lidar_cb,
            10
        )


        self.kiss_config = KISSConfig()
        self.kiss_config.mapping.voxel_size = 1.0
        self.kiss_config.data.max_range = 100.0
        self.odometry = KissICP(self.kiss_config)

        self.closure_config = MapClosuresConfig()
        self.map_closures = MapClosures(self.closure_config)
        self.voxel_local_map = get_voxel_hash_map(self.kiss_config)
        self.current_local_map_pose = np.eye(4)
        self.localized_pose = np.eye(4)
        self.initial_odometry_pose = np.eye(4)
        self.num_maps_in_db = 0
        self.map_idx = 0
        self.is_localized = False

        self.map_db_path = '/navigator/data/slam_results/'
        local_maps_dir = os.path.join(self.map_db_path, "local_maps")
        self.local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))
        poses_file = os.path.join(self.map_db_path, "optimized_poses.txt")
        self.local_map_poses = np.loadtxt(poses_file)
        self.global_map_downsample = None
        self.load_database()
        self.load_global_map()

        self.map_range = self.closure_config.local_map_factor * self.kiss_config.data.max_range

        self.tf_broadcaster = TransformBroadcaster(self)


    def load_global_map(self):
        self.get_logger().info('Loading global map...')

        global_map_points = []
        for (local_map_path, pose) in zip(self.local_map_files, self.local_map_poses):
            local_map = o3d.io.read_point_cloud(local_map_path)
            #local_map = local_map.voxel_down_sample(voxel_size=1.0)
            local_map_pointcloud = np.asarray(local_map.points)
            local_map_pose = pose.reshape(4,4)
            global_map_points.append(transform_points(local_map_pointcloud, local_map_pose))

        # make global map
        global_map_points = np.vstack(global_map_points)
        global_map = o3d.geometry.PointCloud()
        global_map.points = o3d.utility.Vector3dVector(global_map_points)
        
        # get downsampled global map to publish 
        self.global_map_downsample = global_map.voxel_down_sample(voxel_size=1.0)
        self.global_map_downsample = np.asarray(self.global_map_downsample.points)
        dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        self.global_map_downsample = unstructured_to_structured(arr=self.global_map_downsample, dtype=dtypes)

        self.get_logger().info('Finished loading global map!')



    def load_database(self):
        self.get_logger().info('Loading local maps database...')
        for map_idx, local_map_path, in enumerate(self.local_map_files):
            local_map = o3d.io.read_point_cloud(local_map_path)
            local_map_pointcloud = np.asarray(local_map.points)
            self.map_closures.match_and_add(map_idx, local_map_pointcloud)
            self.num_maps_in_db += 1
        self.map_idx = self.num_maps_in_db
        self.get_logger().info('Finished loading local maps database!')
    

    def get_localized_pose(self, pose_estimate, ref_idx, local_map_pcd):
        ref_pcd = o3d.io.read_point_cloud(self.local_map_files[ref_idx])
        ref_local_map_pose = self.local_map_poses[ref_idx].reshape(4, 4)
        query_pcd = o3d.geometry.PointCloud()
        query_pcd.points = o3d.utility.Vector3dVector(local_map_pcd)

        p2p_icp_reg = o3d.pipelines.registration.registration_icp(
            ref_pcd, query_pcd, 0.02, pose_estimate, 
            o3d.pipelines.registration.TransformationEstimationPointToPoint()  
        )

        # draw_registration_result(ref_pcd, query_pcd, p2p_icp_reg.transformation)
        localized_pose = ref_local_map_pose @ np.linalg.inv(p2p_icp_reg.transformation)
        return localized_pose


    def lidar_cb(self, lidar_msg: PointCloud2):
        frame = rnp.numpify(lidar_msg)
        frame = np.array([ frame['x'].flatten(), frame['y'].flatten(), frame['z'].flatten()]).T
        self.odometry.register_frame(frame, None)
        current_frame_pose = self.odometry.poses[-1]
        
        # downsamples current frame and adds it to the current local map
        frame_downsample = voxel_down_sample(frame, self.kiss_config.mapping.voxel_size * 0.5)
        frame_to_local_map_pose = np.linalg.inv(self.current_local_map_pose) @ current_frame_pose
        self.voxel_local_map.add_points(transform_points(frame_downsample, frame_to_local_map_pose))

        if np.linalg.norm(frame_to_local_map_pose[:3, -1]) > self.map_range:

            # add to database, ignore matching
            local_map_pointcloud = self.voxel_local_map.point_cloud()
            print(f'map_index: {self.map_idx}')
            closure = self.map_closures.match_and_add(self.map_idx, local_map_pointcloud)

            if closure.number_of_inliers > self.closure_config.inliers_threshold:
                # only look at matches with the stored map, not the local maps added while localization
                if closure.source_id < self.num_maps_in_db:   
                    self.get_logger().info(f'Localized!!!')
                    self.initial_odometry_pose = self.current_local_map_pose
                    localized_pose = self.get_localized_pose(closure.pose, closure.source_id, local_map_pointcloud)
                    self.localized_pose = localized_pose @ frame_to_local_map_pose
                    self.is_localized = True                        

            # start a new local map initialized with most recent frame's pose as origin
            self.voxel_local_map.remove_far_away_points(frame_to_local_map_pose[:3, -1])
            pts_to_keep = self.voxel_local_map.point_cloud()
            self.voxel_local_map = get_voxel_hash_map(self.kiss_config)
            self.voxel_local_map.add_points(transform_points(pts_to_keep, np.linalg.inv(current_frame_pose) @ self.current_local_map_pose))
            

            self.current_local_map_pose = np.copy(current_frame_pose)
            self.map_idx += 1
    

        # publish global map
        global_map_msg: PointCloud2 = rnp.msgify(PointCloud2, self.global_map_downsample)
        global_map_msg.header.stamp = self.get_clock().now().to_msg()
        global_map_msg.header.frame_id = 'map'
        self.map_publisher.publish(global_map_msg)


        if self.is_localized:
            relative_transform = np.linalg.inv(self.odometry.poses[-2]) @ self.odometry.poses[-1]
            self.localized_pose = self.localized_pose @ relative_transform

            current_pose = self.localized_pose

            # publish odometry message
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = current_pose[:3, -1]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = Rotation.from_matrix(current_pose[:3, :3]).as_quat()
            odom_msg: Odometry = Odometry()
            odom_msg.child_frame_id = 'vehicle'
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.pose.pose = pose
            self.odom_publisher.publish(odom_msg)

            # publish transform from vehicle to map
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'vehicle'

            t.transform.translation.x = current_pose[0, -1]
            t.transform.translation.y = current_pose[1, -1]
            t.transform.translation.z = current_pose[2, -1]
            
            r = Rotation.from_matrix(current_pose[:3, :3]).as_quat()
            t.transform.rotation.x = r[0]
            t.transform.rotation.y = r[1]
            t.transform.rotation.z = r[2]
            t.transform.rotation.w = r[3]

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()