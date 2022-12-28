/*
 * Package:   mapping
 * Filename:  OctoSlamNode.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "mapping/ParticleFilter.hpp"

// Message definitions
#include <carla_msgs/msg/carla_world_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

namespace navigator
{
  namespace perception
  {

    class OctoSlamNode : public rclcpp::Node
    {
    public:
      OctoSlamNode();
      virtual ~OctoSlamNode();

    private:
      // Constants
      // TODO: Convert to ROS parameters
      const double OCTREE_RESOLUTION = 0.2; // meters
      std::chrono::milliseconds MAP_UPDATE_PERIOD = 500ms;
      const std::string MAP_SAVE_PATH = "/navigator/data/maps/";
      const double HIGH_RES_DISTANCE = 10; // meters
      const double MED_RES_DISTANCE = 40;  // meters
      const std::string INITIAL_GUESS_ODOM_TOPIC = "/odometry/gnss_smoothed";

      // Publishers
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_marker_pub;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particle_viz_pub;

      // Subscribers
      rclcpp::Subscription<carla_msgs::msg::CarlaWorldInfo>::SharedPtr world_info_sub;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_odom_sub;
      rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;

      // Callbacks
      void initialOdomCb(nav_msgs::msg::Odometry::SharedPtr msg);
      void pointCloudCb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void worldInfoCb(carla_msgs::msg::CarlaWorldInfo::SharedPtr msg);

      // Timers
      rclcpp::TimerBase::SharedPtr map_marker_timer;

      rosgraph_msgs::msg::Clock clock;
      std::string map_name;
      geometry_msgs::msg::TransformStamped map_bl_transform;
      std::shared_ptr<navigator::perception::ParticleFilter> filter;
      std::shared_ptr<octomap::OcTree> tree;

      std::string getFilenameFromMapName();
      octomap::Pointcloud pclToOctreeCloud(pcl::PointCloud<pcl::PointXYZI> inputCloud);
      void publishMapMarker();
      void saveOctreeBinary();

      std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    };

  }
}
