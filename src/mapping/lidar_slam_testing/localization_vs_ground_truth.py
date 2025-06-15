import math
import numpy as np
from pathlib import Path
import sys
import argparse
import open3d as o3d
from tqdm import tqdm

from sensor_msgs.msg import PointCloud2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from rosbags_utils import to_native
from rosbags.typesys.stores.ros2_humble import (
    builtin_interfaces__msg__Time,
    sensor_msgs__msg__PointCloud2,
    std_msgs__msg__Header,
)

sys.path.append('~/navigator/src/tools/ros2_numpy')
import ros2_numpy as rnp

from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.voxelization import voxel_down_sample


# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)

x0 = 0
y0 = 0
z0 = 0

def run_offline_localization_pipeline(bag_path, lidar_topic_name, gt_topic_name, map_path_str):
  reader = AnyReader([bag_path], default_typestore=typestore)
  if not reader.isopen: reader.open()

  global_map = o3d.io.read_point_cloud(map_path_str)

  kiss_config = KISSConfig()
  kiss_config.mapping.voxel_size = 1.0
  kiss_config.data.max_range = 100.0
  odometry = KissICP(kiss_config, map_path_str)
  full_poses = [odometry.last_pose]

  connections = [x for x in reader.connections if x.topic == lidar_topic_name]
  gtConn = [x for x in reader.connections if x.topic == gt_topic_name]
  num_msgs = connections[0].msgcount
  pbar = tqdm(total=num_msgs)

  lidarmsgs = reader.messages(connections=connections)
  gtmsgs = reader.messages(connections=gtConn)
  first = True
  gps_positions = []

  for connection, _, rawdata in gtmsgs:
    msg = reader.deserialize(rawdata, connection.msgtype)
    if first:
      global x0, y0, z0
      x0 = msg.pose.pose.position.x
      y0 = msg.pose.pose.position.y
      z0 = msg.pose.pose.position.z
      print("x0: " + str(x0))
      print("y0: " + str(y0))
      print("z0: " + str(z0))
      first = False
      gps_positions.append(np.array([0,0,0]))
    
    else:
      # gps_positions.append(gps_to_enu(msg.latitude, msg.longitude, msg.altitude))
      gps_positions.append(np.array([msg.pose.pose.position.x - x0, msg.pose.pose.position.y - y0, msg.pose.pose.position.z - z0]))

  for connection, _, rawdata in lidarmsgs:

    msg = reader.deserialize(rawdata, connection.msgtype)
    pcd = to_native(msg)
    pcd = rnp.numpify(pcd, PointCloud2)
    # Example: assuming `pcd` is a numpy array with shape (N, 4)
    num_points = pcd.shape[0]
    pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T

    # If you don’t have timestamps, create dummy ones
    timestamps = np.linspace(0, 1, num=num_points, dtype=np.float32)

    odometry.register_frame(pcd, timestamps)

    current_pose = odometry.last_pose
    full_poses.append(current_pose)

    pbar.update(1)
    
  trajectory = np.array([pose[:3, 3] for pose in full_poses])  # Extract translation
  # Create lines connecting trajectory points
  lines = [[i, i + 1] for i in range(len(trajectory) - 1)]
  colors = [[1, 0, 0] for _ in lines]  # Red lines
  line_set = o3d.geometry.LineSet()
  line_set.points = o3d.utility.Vector3dVector(trajectory)
  line_set.lines = o3d.utility.Vector2iVector(lines)
  line_set.colors = o3d.utility.Vector3dVector(colors)
  print(trajectory[-1])

  zmax = np.max(gps_positions, axis=0)[2]
  zmin = np.min(gps_positions, axis=0)[2]
  print("Zmax = " + str(zmax))
  print("Zmin = " + str(zmin)) # found that ground truth doesn't reflect z-axis change, at least for now.
  gt_lines = []
  total_displacement = 0.0
  for i in range(len(gps_positions) - 1):
    gt_lines.append([i, i + 1])
    total_displacement += math.sqrt((gps_positions[i+1][0] - gps_positions[i][0]) ** 2
                              +(gps_positions[i+1][1] - gps_positions[i][1]) ** 2
                              +(gps_positions[i+1][2] - gps_positions[i][2]) ** 2)
  gt_colors = [[0, 1, 0] for _ in gt_lines]  # Red lines
  gt_line_set = o3d.geometry.LineSet()
  gt_line_set.points = o3d.utility.Vector3dVector(gps_positions)
  gt_line_set.lines = o3d.utility.Vector2iVector(gt_lines)
  gt_line_set.colors = o3d.utility.Vector3dVector(gt_colors)
  print(gps_positions[-1])

  o3d.visualization.draw_geometries([global_map, line_set, gt_line_set])

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bag_path', help='path to the rosbag to run localization on')
  parser.add_argument('lidar_topic', help='Topic containing lidar messages for localization')
  parser.add_argument('gt_topic', help='Topic containing ground truth messages for comparison')
  parser.add_argument('map_path', help='path to directory with kiss_slam map output')
  args = parser.parse_args()
  bag_path = Path(args.bag_path)
  lidar_topic = str(args.lidar_topic)
  gt_topic = str(args.gt_topic)
  map_path = str(args.map_path)

  run_offline_localization_pipeline(bag_path, lidar_topic, gt_topic, map_path)