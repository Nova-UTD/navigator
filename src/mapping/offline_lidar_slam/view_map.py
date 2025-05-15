import open3d as o3d
import os
import numpy as np
import glob
from offline_lidar_slam import transform_points
import argparse
import os
import math

def view_map(save_dir):
    # local_maps_dir = os.path.join(save_dir, "local_maps")
    # poses_file = os.path.join(save_dir, "poses.txt")
    trajectory_file = os.path.join(save_dir, "full_trajectory.txt")
    gt_file = os.path.join(save_dir, "full_gt.txt")
    # poses = np.loadtxt(poses_file)
    # local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))

    # global_map = []
    # for poses_flattened, local_map_path in zip(poses, local_map_files):
    #     local_map_o3d = o3d.io.read_point_cloud(local_map_path)
    #     local_map_np = np.asarray(local_map_o3d.points)
    #     local_map_pose = poses_flattened.reshape(4, 4)
    #     local_map_np = transform_points(local_map_np, local_map_pose)
    #     global_map.append(local_map_np)

    trajposes = np.loadtxt(trajectory_file).reshape(-1, 4, 4)
    trajectory = np.array([pose[:3, 3] for pose in trajposes])  # Extract translation
    # Create lines connecting trajectory points
    lines = [[i, i + 1] for i in range(len(trajectory) - 1)]
    colors = [[1, 0, 0] for _ in lines]  # Red lines
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(trajectory)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    print(trajectory[-1])

    gt_trajectory = np.loadtxt(gt_file)
    gt_lines = []
    total_displacement = 0.0
    for i in range(len(gt_trajectory) - 1):
      gt_lines.append([i, i + 1])
      total_displacement += math.sqrt((gt_trajectory[i+1][0] - gt_trajectory[i][0]) ** 2
                                +(gt_trajectory[i+1][1] - gt_trajectory[i][1]) ** 2
                                +(gt_trajectory[i+1][2] - gt_trajectory[i][2]) ** 2)
    gt_colors = [[0, 1, 0] for _ in gt_lines]  # Red lines
    gt_line_set = o3d.geometry.LineSet()
    gt_line_set.points = o3d.utility.Vector3dVector(gt_trajectory)
    gt_line_set.lines = o3d.utility.Vector2iVector(gt_lines)
    gt_line_set.colors = o3d.utility.Vector3dVector(gt_colors)
    print(gt_trajectory[-1])

    print("Abs(translation) between ground truth and lidar estimation")
    print("(meters) (dx, dy, dz):")
    print("Beginning:")
    print(str(abs(trajectory[0][0] - gt_trajectory[0][0])) + ", " + str(abs(trajectory[0][1] - gt_trajectory[0][1]))
           + ", " + str(abs(trajectory[0][2] - gt_trajectory[0][2])))
    print("Middle:")
    trajmid = len(trajectory) // 2
    gtmid = len(gt_trajectory) // 2
    print(str(abs(trajectory[trajmid][0] - gt_trajectory[gtmid][0])) + ", " + str(abs(trajectory[trajmid][1] - gt_trajectory[gtmid][1]))
           + ", " + str(abs(trajectory[trajmid][2] - gt_trajectory[gtmid][2])))
    print("End:")
    totxdrift = abs(trajectory[-1][0] - gt_trajectory[-1][0])
    totydrift = abs(trajectory[-1][1] - gt_trajectory[-1][1])
    totzdrift = abs(trajectory[-1][2] - gt_trajectory[-1][2])
    print(str(totxdrift) + ", " + str(totydrift) + ", " + str(totzdrift))
    
    print("\nTotal Displacement: "+str(total_displacement)+" meters")
    print("Avg. lidar odom. drift (in meters) per meter displacement")
    print("(dx/ds, dy/ds, dz/ds):")
    print(str(totxdrift / total_displacement) + ", " + str(totydrift / total_displacement) + ", " + str(totzdrift / total_displacement))

    # global_map = np.vstack(global_map)
    # global_map_o3d = o3d.geometry.PointCloud()
    # global_map_o3d.points = o3d.utility.Vector3dVector(global_map)
    # global_map_o3d = global_map_o3d.voxel_down_sample(voxel_size=1.0)
    # o3d.visualization.draw_geometries([global_map_o3d, line_set, gt_line_set])
    o3d.visualization.draw_geometries([line_set, gt_line_set])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('save_dir', help='directory to which contains the map built by SLAM')
    args = parser.parse_args()
    save_dir = args.save_dir
    assert os.path.isdir(save_dir)

    view_map(save_dir)
    