/*
 * Package:   mapping
 * Filename:  OctreeMappingNode.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace navigator
{
  namespace perception
  {

    class OctreeMappingNode : public rclcpp::Node
    {
    public:
      OctreeMappingNode();
      virtual ~OctreeMappingNode();

    private:
      // std::unique_ptr<navigator::can_interface::CanBus> can_bus;
      // rclcpp::TimerBase::SharedPtr incoming_message_timer;
      // rclcpp::Publisher<voltron_msgs::msg::CanFrame>::SharedPtr incoming_message_publisher;
      void point_cloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
      rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
      rosgraph_msgs::msg::Clock clock;
      octomap::OcTree tree = octomap::OcTree(0.2);

      octomap::Pointcloud pclToOctreeCloud(pcl::PointCloud<pcl::PointXYZI> inputCloud);

      std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    };

  }
}
