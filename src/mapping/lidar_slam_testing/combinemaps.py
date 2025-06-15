import glob
import open3d as o3d
import numpy as np

local_maps_dir = "slam_output/2025-05-20_19-31-34/local_maps/plys"
local_map_files = sorted(glob.glob(local_maps_dir + '/*.ply'))

combined_pcd = o3d.geometry.PointCloud()

for local_map_file in local_map_files:
    pcd = o3d.io.read_point_cloud(local_map_file)
    combined_pcd += pcd

combined_pcd = combined_pcd.voxel_down_sample(voxel_size=0.5)

o3d.io.write_point_cloud("combined_map.pcd", combined_pcd)
print("✅ Saved combined map to combined_map.pcd") 