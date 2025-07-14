import open3d as o3d
import os
import numpy as np
import glob
import argparse
import os
import matplotlib.pyplot as plt


def transform_points(pcd, T):
    R = T[:3, :3]
    t = T[:3, -1]
    return pcd @ R.T + t


def draw_odometry(pred_poses):
    x_pred = pred_poses[:, 0, -1]
    y_pred = pred_poses[:, 1, -1]
    plt.plot(-y_pred, x_pred, color='blue')
    plt.show()

def view_map(save_dir):
    local_maps_dir = os.path.join(save_dir, "local_maps")
    poses_file = os.path.join(save_dir, "optimized_poses.txt")
    poses = np.loadtxt(poses_file)
    local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))

    poses_pred = []
    for pose in poses:
        poses_pred.append(pose.reshape(4,4))
    
    draw_odometry(np.array(poses_pred))
    return
    
    global_map = []
    for poses_flattened, local_map_path in zip(poses, local_map_files):
        local_map_o3d = o3d.io.read_point_cloud(local_map_path)
        local_map_np = np.asarray(local_map_o3d.points)
        local_map_pose = poses_flattened.reshape(4, 4)
        local_map_np = transform_points(local_map_np, local_map_pose)
        global_map.append(local_map_np)

    global_map = np.vstack(global_map)
    global_map_o3d = o3d.geometry.PointCloud()
    global_map_o3d.points = o3d.utility.Vector3dVector(global_map)
    global_map_o3d = global_map_o3d.voxel_down_sample(voxel_size=1.0)
    o3d.visualization.draw_geometries([global_map_o3d])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('save_dir', help='directory to which contains the map built by SLAM')
    args = parser.parse_args()
    save_dir = args.save_dir
    assert os.path.isdir(save_dir)

    view_map(save_dir)
    