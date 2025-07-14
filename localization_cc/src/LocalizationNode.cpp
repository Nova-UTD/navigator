#include "localization/LocalizationNode.hpp"

#include <absl/status/status.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/make_shared.hpp>
#include <memory>
#include <mutex>
#include <thread>

#include "localization/Utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using ::nav_msgs::msg::Odometry;
using ::sensor_msgs::msg::Imu;
using ::sensor_msgs::msg::PointCloud2;

const float kVoxelSize = 0.5f;

namespace navigator {
namespace localization {

LocalizationNode::LocalizationNode()
    : Node("localization_node"),
      global_map_(kVoxelSize),
      viz_(),
      sac_ia_params_{},
      imu_data_(),
      imu_data_mutex_() {
  /* Parameters */
  visualize_global_map_ = utils::declare_and_get_parameter<bool>(
      this, true, "visualize_global_map", "Time to visualize the global map",
      true);

  local_maps_directory_ = utils::declare_and_get_parameter<std::string>(
      this, true, "local_maps_directory",
      "Directory containing local maps (.ply files)", "");

  local_maps_pose_file_ = utils::declare_and_get_parameter<std::string>(
      this, true, "local_maps_pose_file", "File containing local map poses",
      "");

  fpfh_features_file_ = utils::declare_and_get_parameter<std::string>(
      this, true, "fpfh_features_file",
      "File containing FPFH features for the global map", "");

  // SAC-IA parameters
  sac_ia_params_.max_correspondence_distance =
      utils::declare_and_get_parameter<double>(
          this, true, "max_correspondence_distance",
          "Maximum correspondence distance for SAC-IA", 1.0);

  sac_ia_params_.similarity_threshold =
      utils::declare_and_get_parameter<double>(
          this, true, "similarity_threshold", "Similarity threshold for SAC-IA",
          0.5);

  sac_ia_params_.inlier_fraction = utils::declare_and_get_parameter<double>(
      this, true, "inlier_fraction", "Inlier fraction for SAC-IA", 0.25);

  sac_ia_params_.max_iterations = utils::declare_and_get_parameter<int>(
      this, true, "max_iterations", "Maximum iterations for SAC-IA", 10000);

  sac_ia_params_.num_samples = utils::declare_and_get_parameter<int>(
      this, true, "num_samples", "Number of samples for SAC-IA", 3);

  sac_ia_params_.correspondence_randomness =
      utils::declare_and_get_parameter<int>(
          this, true, "correspondence_randomness",
          "Correspondence randomness for SAC-IA", 2);

  /* Publishers*/
  local_map_publisher_ =
      this->create_publisher<PointCloud2>("/localization/point_cloud", 10);
  odometry_publisher_ =
      this->create_publisher<Odometry>("/localization/odometry", 10);

  /* Subscriptions*/
  sensor_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::QoS imu_qos = rclcpp::SensorDataQoS().best_effort().keep_last(10);
  rclcpp::SubscriptionOptions imu_sub_options;
  imu_sub_options.callback_group = sensor_callback_group_;
  imu_subscription_ = this->create_subscription<Imu>(
      "/ouster/imu", imu_qos,
      std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1),
      imu_sub_options);

  rclcpp::QoS lidar_qos = rclcpp::SensorDataQoS().best_effort().keep_last(20);
  rclcpp::SubscriptionOptions lidar_sub_options;
  lidar_sub_options.callback_group = sensor_callback_group_;
  point_cloud_subscription_ = this->create_subscription<PointCloud2>(
      "/lidar", lidar_qos,
      std::bind(&LocalizationNode::pointCloudCallback, this,
                std::placeholders::_1),
      lidar_sub_options);

  /* TF listeners/broadcasters */
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load global map.
  absl::Status load_status =
      fpfh_features_file_.empty()
          ? global_map_.loadMapAndExtractFeatures(local_maps_directory_,
                                                  local_maps_pose_file_)
          : global_map_.loadMapAndFeatures(local_maps_directory_,
                                           local_maps_pose_file_,
                                           fpfh_features_file_);
  if (!load_status.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map with error: %s",
                 std::string(load_status.message()).c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Global map loaded successfully.");

  if (visualize_global_map_) {
    viz_.run();
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(global_map_.getCloud());
    crop_box.setMin(Eigen::Vector4f(-10, -10, -10, 1));
    crop_box.setMax(Eigen::Vector4f(100, 100, 100, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cropped_cloud);
    viz_.update("global_map", cropped_cloud);
  }
}

void LocalizationNode::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  std::lock_guard<std::mutex> lock(imu_data_mutex_);
  imu_data_.push_back(imu_msg);
}

void LocalizationNode::pointCloudCallback(
    PointCloud2::ConstSharedPtr point_cloud_msg) {
  std::lock_guard<std::mutex> lock(imu_data_mutex_);
  if (!global_map_.hasLoaded()) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Global map not loaded yet, skipping scan processing.");
    return;
  }

  if (localized_) {
    RCLCPP_DEBUG(this->get_logger(), "Already localized, skipping scan.");
    return;
  }
  // Convert ROS point cloud to PCL point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *current_scan);

  // Ensure that the point cloud does not contain NaNs.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*current_scan, *current_scan, indices);

  // Downsample the point cloud at same voxel size as global map.
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(current_scan);
  voxel_grid.setLeafSize(kVoxelSize, kVoxelSize, kVoxelSize);
  voxel_grid.filter(*current_scan);

  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr current_scan_features =
      utils::computeFPFH(current_scan);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map_pcd =
      global_map_.getCloud();
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr global_map_features =
      global_map_.getFPFHFeatures();

  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ,
                                   pcl::FPFHSignature33>
      sac_ia;
  sac_ia.setInputSource(current_scan);
  sac_ia.setSourceFeatures(current_scan_features);
  sac_ia.setInputTarget(global_map_pcd);
  sac_ia.setTargetFeatures(global_map_features);
  sac_ia.setMaximumIterations(sac_ia_params_.max_iterations);
  sac_ia.setNumberOfSamples(sac_ia_params_.num_samples);
  sac_ia.setCorrespondenceRandomness(sac_ia_params_.correspondence_randomness);
  sac_ia.setSimilarityThreshold(sac_ia_params_.similarity_threshold);
  sac_ia.setMaxCorrespondenceDistance(
      sac_ia_params_.max_correspondence_distance);
  sac_ia.setInlierFraction(sac_ia_params_.inlier_fraction);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan(
      new pcl::PointCloud<pcl::PointXYZ>);
  sac_ia.align(*aligned_scan);
  RCLCPP_INFO(this->get_logger(), "ICP has converged with score %f",
              sac_ia.getFitnessScore());

  if (sac_ia.hasConverged()) {
    RCLCPP_INFO(this->get_logger(), "sac_ia converged with score %f ",
                sac_ia.getFitnessScore());
    RCLCPP_INFO(this->get_logger(), "sac_ia transformation: \n %s",
                sac_ia.getFinalTransformation().format(Eigen::IOFormat()));
  } else {
    RCLCPP_INFO(this->get_logger(), "sac_ia did not converge");
  }
}
}  // namespace localization

}  // namespace navigator
