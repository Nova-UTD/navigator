#pragma once

#include <absl/strings/string_view.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

namespace navigator {

namespace localization {

namespace utils {

/**
 * Declare a parameter and get its value from the node.
 *
 * @param node Node to declare the parameter on.
 * @param read_only Whether the parameter is read-only.
 * @param name Name of the parameter.
 * @param description Description of the parameter.
 * @param default_value Default value of the parameter.
 *
 * @return Value of the parameter.
 */
template <typename T>
T declare_and_get_parameter(rclcpp::Node *node, bool read_only,
                            absl::string_view name,
                            absl::string_view description,
                            const T &default_value) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.read_only = read_only;
  desc.description = description.data();
  node->declare_parameter(name.data(), default_value, desc);
  return node->get_parameter(name.data()).get_value<T>();
}

/**
 * Compute Fast Point Feature Histograms (FPFH) for a given point cloud.
 *
 * @param cloud Point cloud to compute FPFH for.
 * @param normal_estimation_radius Sphere radius to determine nearest neighbor
 * in normal estimation.
 * @param feature_estimation_radius Sphere radius to determine nearest neighbor
 * in feature estimation.
 *
 * @return FPFH features for the input point cloud.
 */
pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr computeFPFH(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const double normal_estimation_radius = 2.0,
    const double feature_estimation_radius = 8.0);

}  // namespace utils

}  // namespace localization

}  // namespace navigator