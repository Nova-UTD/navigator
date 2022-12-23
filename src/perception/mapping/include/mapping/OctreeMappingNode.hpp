/*
 * Package:   mapping
 * Filename:  OctreeMappingNode.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

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
      octomap::OcTree tree = octomap::OcTree(0.2);
    };

  }
}
