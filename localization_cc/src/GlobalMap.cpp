#include "localization/GlobalMap.hpp"

#include <absl/status/status.h>
#include <absl/status/statusor.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_split.h>
#include <absl/strings/string_view.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <ouster/point_viz.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "localization/Utils.hpp"
#include "rclcpp/rclcpp.hpp"

const rclcpp::Logger kLogger = rclcpp::get_logger("global_map_loader");

absl::StatusOr<std::vector<Eigen::Matrix4d>> loadPoses(
    absl::string_view local_map_pose_file) {
  RCLCPP_INFO(kLogger, "Loading poses from %s", local_map_pose_file.data());

  // Check that the pose file exists.
  if (!std::filesystem::is_regular_file(local_map_pose_file.data())) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid pose file '%s'", local_map_pose_file.data()));
  }

  // Read all map poses into memory.
  std::ifstream pose_file = std::ifstream(local_map_pose_file.data());
  std::stringstream pose_content;
  pose_content << pose_file.rdbuf();

  /* Parse map poses into 4x4 matrices. */
  std::vector<Eigen::Matrix4d> poses;
  // Split file content by line.
  std::vector<std::string> pose_content_split =
      absl::StrSplit(pose_content.str(), '\n', absl::SkipEmpty());
  for (const auto &line : pose_content_split) {
    // Split each line by space.
    std::vector<std::string> pose_values =
        absl::StrSplit(line, ' ', absl::SkipEmpty());

    // Check if the line has 16 values.
    if (pose_values.size() != 16) {
      return absl::InvalidArgumentError(
          "Invalid pose with invalid number of values");
    }

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < 16; i++) {
      int row = i / 4, col = i % 4;
      pose(row, col) = std::stod(pose_values[i]);
    }
    poses.push_back(pose);
  }

  return poses;
}

absl::StatusOr<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadLocalMapFromPLY(
    absl::string_view local_map_file, const Eigen::Matrix4d &pose) {
  RCLCPP_DEBUG(kLogger, "Loading local map from %s", local_map_file.data());

  if (!std::filesystem::is_regular_file(local_map_file.data())) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid local map file %s", local_map_file.data()));
  }

  // Load local map using Open3D and convert it to PCL.
  // This is a current workaround to handle doubles stored in PLY files.
  std::shared_ptr<open3d::geometry::PointCloud> open3d_local_map =
      open3d::io::CreatePointCloudFromFile(local_map_file.data());
  if (open3d_local_map == nullptr || open3d_local_map->IsEmpty()) {
    return absl::InternalError(
        absl::StrFormat("Failed to load local map %s", local_map_file.data()));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : open3d_local_map->points_) {
    local_map->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }

  // Transform local map to global map frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*local_map, *local_map_transformed,
                           pose.cast<float>());

  return local_map_transformed;
}

absl::StatusOr<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadGlobalMapFromPLYs(
    absl::string_view local_map_directory,
    const std::vector<Eigen::Matrix4d> &poses) {
  RCLCPP_INFO(kLogger, "Loading global map from %s",
              local_map_directory.data());

  // Check that the local map directory exists.
  if (!std::filesystem::is_directory(local_map_directory.data())) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Invalid local map directory '%s'", local_map_directory.data()));
  }

  // Load all local maps in the directory.
  auto local_map_files_it =
      std::filesystem::directory_iterator(local_map_directory.data());
  std::vector<std::string> local_map_files;
  for (const auto &entry : local_map_files_it) {
    local_map_files.push_back(entry.path().string());
  }

  // TODO: improve the storage of local maps because files can be loaded out of
  // order, which will cause the poses to be mismatched. This is a temporary fix
  // to sort the files by name (they are numbered for now).
  std::sort(local_map_files.begin(), local_map_files.end());

  // Ensure that each map has a corresponding pose.
  if (poses.size() != local_map_files.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Different number of poses than local maps, got %lu poses and %lu maps",
        poses.size(), local_map_files.size()));
  }

  /* Load each map concurrently and add it to a global map */
  std::vector<std::future<absl::StatusOr<pcl::PointCloud<pcl::PointXYZ>::Ptr>>>
      local_map_futures;

  // Load each local map in parallel.
  for (long unsigned int i = 0; i < local_map_files.size(); i++) {
    local_map_futures.push_back(std::async(std::launch::async,
                                           &loadLocalMapFromPLY,
                                           local_map_files[i], poses[i]));
  }

  // Wait for all local maps to be loaded. If any fail, return the error.
  // Add all local maps to the global map.
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto &future : local_map_futures) {
    absl::StatusOr<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_map_status =
        future.get();
    if (!local_map_status.ok()) {
      return local_map_status.status();
    }
    *global_map += *local_map_status.value();
  }

  return global_map;
}

namespace navigator {
namespace localization {
GlobalMap::GlobalMap(float voxel_size)
    : global_map_(), voxel_size_(voxel_size) {}

absl::Status GlobalMap::loadMap(absl::string_view local_map_directory,
                                absl::string_view local_map_pose_file) {
  /* Load poses */
  absl::StatusOr<std::vector<Eigen::Matrix4d>> poses_or =
      loadPoses(local_map_pose_file);
  if (!poses_or.ok()) {
    return poses_or.status();
  }
  std::vector<Eigen::Matrix4d> poses = poses_or.value();

  /* Load global map */
  absl::StatusOr<pcl::PointCloud<pcl::PointXYZ>::Ptr> global_map_or =
      loadGlobalMapFromPLYs(local_map_directory, poses);
  if (!global_map_or.ok()) {
    return global_map_or.status();
  }
  global_map_ = global_map_or.value();
  size_t cloud_size = global_map_->size();

  /* Downsample global map */
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(global_map_);
  voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  voxel_grid.filter(*global_map_);

  if (global_map_->empty()) {
    return absl::InternalError("Downsampled global map is empty");
  }
  RCLCPP_DEBUG(kLogger, "Downsampled global map from %lu to %lu points",
               cloud_size, global_map_->size());

  return absl::OkStatus();
}

absl::Status GlobalMap::loadMapAndExtractFeatures(
    absl::string_view local_map_directory,
    absl::string_view local_map_pose_file) {
  /* Load the global map */
  absl::Status load_status = loadMap(local_map_directory, local_map_pose_file);
  if (!load_status.ok()) {
    return load_status;
  }

  /* Compute FPFH features for the global map */
  RCLCPP_INFO(kLogger,
              "Computing FPFH features for the global map, this may "
              "take a while...");
  fpfh_features_ = utils::computeFPFH(global_map_, 2.0, 8.0);
  pcl::io::savePCDFileBinary("global_map_features.pcd", *fpfh_features_);

  return absl::OkStatus();
}

absl::Status GlobalMap::loadMapAndFeatures(
    absl::string_view local_map_directory,
    absl::string_view local_map_pose_file,
    absl::string_view fpfh_features_file) {
  /* Load the global map */
  absl::Status load_status = loadMap(local_map_directory, local_map_pose_file);
  if (!load_status.ok()) {
    return load_status;
  }

  /* Load FPFH features */
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(
      new pcl::PointCloud<pcl::FPFHSignature33>);
  if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(fpfh_features_file.data(),
                                                 *features) == -1) {
    return absl::InternalError(absl::StrFormat(
        "Failed to load FPFH features from %s", fpfh_features_file.data()));
  }
  fpfh_features_ = features;
  RCLCPP_INFO(kLogger, "Loaded global map features from %s",
              fpfh_features_file.data());

  return absl::OkStatus();
}

}  // namespace localization
}  // namespace navigator