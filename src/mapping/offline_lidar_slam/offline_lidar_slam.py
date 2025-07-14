import numpy as np
from pathlib import Path
import os
import sys
import argparse
import open3d as o3d
from tqdm import tqdm

# for ROS
sys.path.append('/navigator/src/tools/ros2_numpy')
import ros2_numpy as rnp
import sensor_msgs
from sensor_msgs.msg import PointCloud2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from rosbags_utils import to_native
from rosbags.typesys.stores.ros2_humble import (
    builtin_interfaces__msg__Time,
    sensor_msgs__msg__PointCloud2,
    std_msgs__msg__Header,
)

# for SLAM
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.voxelization import voxel_down_sample
from map_closures.map_closures import MapClosures
from map_closures.config import MapClosuresConfig
import g2o
from pose_graph_optimizer import PoseGraphOptimizer


# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)


def transform_points(pcd, T):
    R = T[:3, :3]
    t = T[:3, -1]
    return pcd @ R.T + t


def run_offline_slam_pipeline(bag_path, topic_name, save_dir):
    
    reader = AnyReader([bag_path], default_typestore=typestore)
    if not reader.isopen: reader.open()
    
    # for saving files
    local_maps_dir = os.path.join(save_dir, "local_maps")
    os.makedirs(local_maps_dir, exist_ok=True)
    poses_file = os.path.join(save_dir, "poses.txt")

    # odometry
    kiss_config = KISSConfig()
    kiss_config.mapping.voxel_size = 1.0
    kiss_config.data.max_range = 100.0
    odometry = KissICP(kiss_config)

    # loop closure
    closure_config = MapClosuresConfig()
    map_closures = MapClosures(closure_config)

    # necessary variables
    voxel_local_map = get_voxel_hash_map(kiss_config)
    current_local_map_pose = np.eye(4)
    previous_local_map_pose = None
    map_idx = 0
    map_range = 60.0

    pgo = PoseGraphOptimizer()

    connections = [x for x in reader.connections if x.topic == topic_name]
    num_msgs = connections[0].msgcount
    pbar = tqdm(total=num_msgs)

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        pcd = to_native(msg)
        pcd = rnp.numpify(pcd, PointCloud2)
        pcd = np.array([pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten()]).T

        odometry.register_frame(pcd, None)

        # downsamples current scan and adds it to the current local map
        current_frame_pose = odometry.poses[-1]
        scan_downsample = voxel_down_sample(pcd, kiss_config.mapping.voxel_size * 0.5)
        frame_to_local_map_pose = np.linalg.inv(current_local_map_pose) @ current_frame_pose
        voxel_local_map.add_points(transform_points(scan_downsample, frame_to_local_map_pose))

        if np.linalg.norm(frame_to_local_map_pose[:3, -1]) > map_range:
            
            local_map_pointcloud = voxel_local_map.point_cloud()

            o3d.io.write_point_cloud(
                os.path.join(local_maps_dir, f"{map_idx:06d}.ply"),
                o3d.geometry.PointCloud(o3d.utility.Vector3dVector(local_map_pointcloud)),
            )

            # add current local map pose as vertex
            pose = g2o.Isometry3d(current_local_map_pose)
            pgo.add_vertex(map_idx, pose)

            if previous_local_map_pose is not None:
                # add odometry edge
                transform = g2o.Isometry3d(np.linalg.inv(previous_local_map_pose) @ current_local_map_pose)
                pgo.add_edge([map_idx-1, map_idx], transform, robust_kernel=g2o.RobustKernelHuber())

            closure = map_closures.match_and_add(map_idx, local_map_pointcloud)
            if closure.number_of_inliers > closure_config.inliers_threshold:
                #print(f'Loop closure found between: {closure.source_id} (source) and {map_idx} (target) !!!')
                # add loop closure edge
                pose = g2o.Isometry3d(closure.pose)
                pgo.add_edge([closure.target_id, closure.source_id], pose, robust_kernel=g2o.RobustKernelHuber())

            previous_local_map_pose = current_local_map_pose

            # start a new local map initialized with most recent frame as origin
            voxel_local_map.remove_far_away_points(frame_to_local_map_pose[:3, -1])
            pts_to_keep = voxel_local_map.point_cloud()
            voxel_local_map = get_voxel_hash_map(kiss_config)
            voxel_local_map.add_points(
                transform_points(pts_to_keep, np.linalg.inv(current_frame_pose) @ current_local_map_pose)
            )
            current_local_map_pose = np.copy(current_frame_pose)

            map_idx += 1
        
        pbar.update(1)
    
    if reader.isopen: reader.close()
    pbar.close()

    pgo.set_verbose(True)
    pgo.optimize(max_iterations=50)

    optimized_poses = []
    for i in range(map_idx):
        local_map_pose_optimized = pgo.get_pose(i).matrix()
        optimized_poses.append(local_map_pose_optimized.flatten())

    np.savetxt(poses_file, optimized_poses)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help='path to the ROS bag you want to apply lidar SLAM')
    parser.add_argument('topic_name', help='name of the topic containing the lidar messages')
    parser.add_argument('save_dir', help='directory to which SLAM results will be saved to; automatically created if it does not exist')
    args = parser.parse_args()
    bag_path = Path(args.bag_path)
    topic_name = str(args.topic_name)
    save_dir = Path(args.save_dir)
    
    assert os.path.exists(bag_path)
    os.makedirs(save_dir, exist_ok=True)

    run_offline_slam_pipeline(bag_path, topic_name, save_dir)

