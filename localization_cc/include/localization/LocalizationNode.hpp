#pragma once

#include <absl/status/status.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "localization/GlobalMap.hpp"
#include "localization/Visualizer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace navigator {
namespace localization {

struct SacIaParams {
  int max_iterations;
  int num_samples;
  int correspondence_randomness;
  float similarity_threshold;
  float max_correspondence_distance;
  float inlier_fraction;
};

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode();

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      local_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscription_;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool visualize_global_map_;
  std::string local_maps_directory_;
  std::string local_maps_pose_file_;
  std::string fpfh_features_file_;

  GlobalMap global_map_;
  Visualizer viz_;
  SacIaParams sac_ia_params_;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_data_;
  std::mutex imu_data_mutex_;

  bool localized_;
};
}  // namespace localization
}  // namespace navigator