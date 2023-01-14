/*
 * Package:   mapping
 * Filename:  GroundSegmentationNode.hpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once


// Message definitions

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using rosgraph_msgs::msg::Clock
using sensor_msgs::msg::PointCloud2

namespace navigator
{
  namespace perception
  {

    class GroundSegmentationNode : public rclcpp::Node
    {
    public:
      GroundSegmentationNode();
      virtual ~GroundSegmentationNode();

    private:

      // Publishers
      rclcpp::Publisher<PointCloud2>::SharedPtr filtered_lidar_pub;

      // Subscribers
      rclcpp::Subscription<Clock>::SharedPtr clock_sub;
      rclcpp::Subscription<PointCloud2>::SharedPtr raw_lidar_sub;

      // Callbacks
      void pointCloudCb(PointCloud2::SharedPtr msg);

      // Timers
      // rclcpp::TimerBase::SharedPtr map_marker_timer;

      Clock clock;
      
    };

  }
}
