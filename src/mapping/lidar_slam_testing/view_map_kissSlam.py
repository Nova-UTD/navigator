import open3d as o3d
import numpy as np
import glob
import math

kissSlamTrajFile = "slam_output/2025-05-20_19-31-34/rosbag2_2025_05_19-19_43_05_poses_kitti.txt"
local_maps_dir = "slam_output/2025-05-20_19-31-34/local_maps/plys"
local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))
gt_file = "savereal/full_gt.txt"

#apparently, kiss-slam even positions the maps for you! you don't have to do any transformation based on their relative pose.
global_map = []
for local_map_path in local_map_files:
    local_map_o3d = o3d.io.read_point_cloud(local_map_path)
    local_map_np = np.asarray(local_map_o3d.points)
    global_map.append(local_map_np)
  
trajposes = np.loadtxt(kissSlamTrajFile).reshape(-1, 3, 4)
trajectory = np.array([pose[:, 3] for pose in trajposes])  # Extract translation
# Create lines connecting trajectory points
lines = [[i, i + 1] for i in range(len(trajectory) - 1)]
colors = [[1, 0, 0] for _ in lines]  # Red lines
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(trajectory)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)
print(trajectory[-1])

gt_trajectory = np.loadtxt(gt_file)
zmax = np.max(gt_trajectory, axis=0)[2]
zmin = np.min(gt_trajectory, axis=0)[2]
print("Zmax = " + str(zmax))
print("Zmin = " + str(zmin))
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

global_map = np.vstack(global_map)
global_map_o3d = o3d.geometry.PointCloud()
global_map_o3d.points = o3d.utility.Vector3dVector(global_map)
# global_map_o3d = global_map_o3d.voxel_down_sample(voxel_size=1.0)
o3d.visualization.draw_geometries([global_map_o3d, line_set, gt_line_set])
