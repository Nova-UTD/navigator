#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import numpy as np
import os
import signal
import time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import glob
import open3d as o3d
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from kiss_icp.voxelization import voxel_down_sample
from math import sqrt
import yaml
import threading
from lidar_SLAM.keyboardListener import keyboard_listener_thread

BEGIN_PCD = str(os.path.join(get_package_share_directory('lidar_SLAM'),
                   'resource', 'combined_map.pcd'))

VOXEL_SIZE = 0.5

class SlamRunnerNode(Node):
    def __init__(self):
        super().__init__('slam_runner_node')

        self.lidar_topic_ = '/lidar/filtered'

        # Generate a unique name for the bag file based on the current timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_name = f"slam_run_{timestamp}"
        self.resourceDir = os.path.join(get_package_share_directory('lidar_SLAM'), 'resource')
        self.bag_path_ = os.path.join(self.resourceDir, bag_name)
        
        self.slam_out_path = os.path.abspath(os.path.join(self.resourceDir, 'slam_output'))
        self.kiss_config_path_ = os.path.join(self.resourceDir, 'kiss_slam.yaml')

        self.get_logger().info(f"Node initialized. Will record topic '{self.lidar_topic_}'.")
        self.get_logger().info(f"Output bag file will be saved at: '{os.path.abspath(self.bag_path_)}'")
        self.initial_pos_sub = self.create_subscription(PointCloud2, '/lidar/filtered', self.register, 1)
        self.initial_pose = np.eye(4)
        self.localizeCount = 0
        self.bag_process_ = None
        self.begin_pcd = o3d.io.read_point_cloud(BEGIN_PCD)
        self.begin_pcd = self.begin_pcd.voxel_down_sample(VOXEL_SIZE)
        self.first = True
        self.gotPoseMessage = True
        self.kiss_config = KISSConfig()
        self.kiss_config.mapping.voxel_size = VOXEL_SIZE
        self.odometry = KissICP(self.kiss_config, BEGIN_PCD)
        self.initPoseOdomtery = KissICP(self.kiss_config)
        self.initPCD = None

    def start_recording(self):
        """
        Starts the 'ros2 bag record' process as a subprocess.
        """
        command = [
            'ros2', 'bag', 'record',
            '-o', self.bag_path_,
            self.lidar_topic_
        ]
        self.get_logger().info(f"Starting rosbag recording with command: {' '.join(command)}")
        
        # Using preexec_fn=os.setsid creates a new process group.
        # This allows us to send a signal to the entire process group,
        # ensuring that the ros2 bag command and any of its children
        # receive the signal and shut down gracefully.
        self.bag_process_ = subprocess.Popen(command, preexec_fn=os.setsid)
        self.get_logger().info(f"Bag recording process started with PID: {self.bag_process_.pid}")

    def register(self, pcd):
      if self.localizeCount < 10:
        pcd = rnp.numpify(pcd, PointCloud2)
        num_points = pcd.shape[0]
        pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T
        if self.initPCD is None: self.initPCD = pcd

        # global registration for initial pose
        if self.first:
          timestamps = np.linspace(0, 1, num=num_points, dtype=np.float32)
          self.initPoseOdomtery.register_frame(pcd, timestamps)
          pcd = self.initPoseOdomtery.local_map.point_cloud()
          target = self.begin_pcd
          o3d_pcd = o3d.geometry.PointCloud()
          o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
          o3d_pcd = o3d_pcd.remove_non_finite_points(remove_nan=True, remove_infinite=True)
          o3d_pcd = o3d_pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
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
          
          distance_threshold = VOXEL_SIZE * 2
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
        timestamps = np.linspace(0, 1, num=self.initPCD.shape[0], dtype=np.float32)

        self.odometry.register_frame(self.initPCD, timestamps)

        self.initial_pose = self.odometry.last_pose
        self.localizeCount += 1

      elif self.gotPoseMessage:
          self.get_logger().info("Determined initial pose. Press 's' to save when mapping complete.")
          self.gotPoseMessage = False
          listener = threading.Thread(target=keyboard_listener_thread, args=(self,), daemon=True)
          listener.start()
          
    def stop_recording(self):
        """
        Gracefully stops the rosbag recording process by sending a SIGINT signal.
        """
        if self.bag_process_ and self.bag_process_.poll() is None:
            print("Stopping rosbag recording process...")
            
            # Send SIGINT (Ctrl+C) to the process group to trigger a clean shutdown
            os.killpg(os.getpgid(self.bag_process_.pid), signal.SIGINT)
            
            # Wait for the process to terminate
            self.bag_process_.wait()
            print("Rosbag recording stopped and file finalized.")
        else:
            print("Bag recording process was not running or already stopped.")

    def run_slam(self):
        """
        Launches the KISS-ICP pipeline using the recorded bag file.
        """
        data = {}
        if self.kiss_config_path_ and os.path.exists(self.kiss_config_path_):
            try:
              with open(self.kiss_config_path_, 'r') as file:
                loaded_data = yaml.safe_load(file)
                if loaded_data is not None:
                    data = loaded_data
                print(f"File '{self.kiss_config_path_}' loaded successfully.")
            except yaml.YAMLError as e:
                print(f"Error parsing YAML from {self.kiss_config_path_}: {e}")
            except Exception as e:
                print(f"An unexpected error occurred while loading {self.kiss_config_path_}: {e}")
                return
        else:
            print("No base KISS config file provided. Creating one with just the output directory.")

        data["out_dir"] = self.slam_out_path
        data["keypose"] = self.initial_pose.tolist()
        data["pcdPath"] = BEGIN_PCD

        try:
          with open(self.kiss_config_path_, 'w') as file:
              yaml.dump(data, file, default_flow_style=False, indent=4)
          print(f"File '{self.kiss_config_path_}' updated successfully.")
        except IOError as e:
            print(f"Error writing to file {self.kiss_config_path_}: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while writing to {self.kiss_config_path_}: {e}")

        print(f"Wrote SLAM config: {self.kiss_config_path_}")
        print(f"SLAM output will be saved to: {self.slam_out_path}")

        print("Starting KISS-SLAM pipeline...")
        # NOTE: The path to the bag file for kiss_icp is the directory itself.
        command = [
            'kiss_slam_pipeline',
            '--config', os.path.abspath(self.kiss_config_path_),
            '--topic', self.lidar_topic_,
            os.path.abspath(self.bag_path_)
        ]
        print(f"Executing SLAM command: {' '.join(command)}")

        try:
            # Use subprocess.run as we want to wait for it to complete.
            result = subprocess.run(command, check=True)
            if result.returncode == 0:
                print("KISS-SLAM pipeline finished successfully.")
            else:
                 print(f"KISS-SLAM pipeline exited with error code {result.returncode}.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to execute KISS-SLAM: {e}")
        except FileNotFoundError:
            print("The 'kiss_slam_pipeline' command was not found. Is kiss_slam installed?")

        local_maps_dir = os.path.join(self.slam_out_path, 'latest', 'local_maps', 'plys')
        local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))

        combined_pcd = o3d.geometry.PointCloud()

        for local_map_file in local_map_files:
            pcd = o3d.io.read_point_cloud(local_map_file)
            combined_pcd += pcd

        print(f"path: {BEGIN_PCD}")
        combined_pcd += self.begin_pcd
        combined_pcd = combined_pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)

        o3d.io.write_point_cloud(BEGIN_PCD, combined_pcd)
        o3d.io.write_point_cloud('combined_map.pcd', combined_pcd)
    
    def trigger_shutdown_and_slam(self):
        self.stop_recording()
          
        # Give a moment for the file system to catch up if needed
        time.sleep(1)

        # 2. Run the SLAM pipeline on the completed bag file.
        self.run_slam()

        # 3. Clean up the node.
        self.destroy_node()

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlamRunnerNode()
        node.start_recording()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, initiating shutdown sequence.")
        node.stop_recording()
        node.destroy_node()

if __name__ == '__main__':
    main()
