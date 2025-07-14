#include "localization/Utils.hpp"

#include <absl/strings/string_view.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

namespace navigator {

namespace localization {

namespace utils {

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr computeFPFH(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const double normal_estimation_radius,
    const double feature_estimation_radius) {
  // Extract surface normals for incoming scan.
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setRadiusSearch(normal_estimation_radius);
  norm_est.setInputCloud(cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  norm_est.compute(*normals);

  // Extra FPFH for incoming scan.
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      feat_est;
  feat_est.setRadiusSearch(feature_estimation_radius);
  feat_est.setInputCloud(cloud);
  feat_est.setInputNormals(normals);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(
      new pcl::PointCloud<pcl::FPFHSignature33>);
  feat_est.compute(*features);

  return features;
}

}  // namespace utils

}  // namespace localization

}  // namespace navigator