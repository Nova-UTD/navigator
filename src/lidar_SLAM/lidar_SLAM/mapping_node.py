#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import numpy as np
import os
import signal
import time
from datetime import datetime
import ament_index_python
from ament_index_python.packages import get_package_share_directory
import glob
import open3d as o3d
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations
from math import sqrt
import yaml

class SlamRunnerNode(Node):
    """
    A ROS2 node that automates the process of SLAM data collection and execution.

    This node performs the following steps:
    1. On startup, it begins recording a rosbag of a specified LiDAR topic.
    2. It waits until the user signals shutdown (Ctrl+C).
    3. On shutdown, it gracefully stops the rosbag recording, ensuring the bag
       file is properly saved and closed.
    4. It then immediately launches the KISS-ICP SLAM pipeline, feeding it the
       bag file that was just recorded.
    """
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
        self.initial_pos_sub = self.create_subscription(Odometry, '/gnss_gt/odometry', self.initialPose, 1)
        # self.initial_imu_sub = self.create_subscription(Imu, '/imu', self.initialOrient, 1)
        self.initial_pose = np.eye(4)
        self.second_pose = np.eye(4)
        self.initial_pos_gathered = False
        self.initial_pose_determined = False
        self.bag_process_ = None

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

    def initialPose(self, gnssgt):
      if not self.initial_pos_gathered:
        self.initial_pose[0][3] = gnssgt.pose.pose.position.x
        self.initial_pose[1][3] = gnssgt.pose.pose.position.y
        self.initial_pose[2][3] = gnssgt.pose.pose.position.z
        self.get_logger().info("Drive directly forward...")
        self.initial_pos_gathered = True
      
      elif not self.initial_pose_determined:
        if ((gnssgt.pose.pose.position.x - self.initial_pose[0][3]) ** 2 + 
            (gnssgt.pose.pose.position.y - self.initial_pose[1][3]) ** 2 + 
            (gnssgt.pose.pose.position.z - self.initial_pose[2][3]) ** 2) > 25:
          self.second_pose[0][3] = gnssgt.pose.pose.position.x
          self.second_pose[1][3] = gnssgt.pose.pose.position.y
          self.second_pose[2][3] = gnssgt.pose.pose.position.z
          translationX = self.second_pose[0][3] - self.initial_pose[0][3]
          translationY = self.second_pose[1][3] - self.initial_pose[1][3]
          hyp = sqrt(translationX ** 2 + translationY ** 2)
          sinTheta = translationY / hyp
          cosTheta = translationX / hyp
          self.initial_pose[0][0] = cosTheta
          self.initial_pose[1][1] = cosTheta
          self.initial_pose[1][0] = sinTheta
          self.initial_pose[0][1] = -1 * sinTheta
          self.get_logger().info("Initial pose determined.")
          self.initial_pose_determined = True
          
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

        # combined_pcd.transform(self.initial_pose)
        combined_pcd = combined_pcd.voxel_down_sample(voxel_size=0.5)

        pcdpath = os.path.join(self.resourceDir, 'combined_map.pcd')
        o3d.io.write_point_cloud(pcdpath, combined_pcd)
        o3d.io.write_point_cloud('combined_map.pcd', combined_pcd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlamRunnerNode()
        node.start_recording()
        
        # Keep the node alive, waiting for Ctrl+C
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, initiating shutdown sequence.")
    
    finally:
        # The shutdown sequence is placed in 'finally' to ensure it runs
        # even if other errors occur.
        if node:
            # 1. Stop the recording first to finalize the bag file.
            node.stop_recording()
            
            # Give a moment for the file system to catch up if needed
            time.sleep(1)

            # 2. Run the SLAM pipeline on the completed bag file.
            node.run_slam()

            # 3. Clean up the node.
            node.destroy_node()

if __name__ == '__main__':
    main()
