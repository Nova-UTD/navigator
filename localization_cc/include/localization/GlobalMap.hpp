#pragma once

#include <absl/status/status.h>
#include <absl/status/statusor.h>
#include <absl/strings/string_view.h>
#include <ouster/point_viz.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <vector>

namespace navigator {
namespace localization {
class GlobalMap {
 public:
  GlobalMap(float voxel_size);

  absl::Status loadMapAndExtractFeatures(absl::string_view local_map_directory,
                    absl::string_view local_map_pose_file);

  absl::Status loadMapAndFeatures(absl::string_view local_map_directory,
                    absl::string_view local_map_pose_file,
                    absl::string_view fpfh_features_file);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getCloud() { return global_map_; }

  bool hasLoaded() {
    return global_map_.get() != nullptr && !global_map_->empty() &&
           fpfh_features_.get() != nullptr && !fpfh_features_->empty();
  }

  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr getFPFHFeatures() {
    return fpfh_features_;
  }

 private:
  /**
   * Loads the global map from a directory containing PLY files and a Numpy pose
   * file.
   *
   * @param local_map_directory Directory containing PLY files.
   * @param local_map_pose_file File containing poses for each PLY file.
   *
   * @return Status of the load operation.
   */
  absl::Status loadMap(absl::string_view local_map_directory,
                            absl::string_view local_map_pose_file);

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  // Voxel size for downsampling the global map.
  // Should be in same units as the point cloud.
  float voxel_size_;

  // FPFH features for the global map.
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfh_features_;

  std::shared_ptr<ouster::viz::PointViz> viz_;
};
}  // namespace localization
}  // namespace navigator