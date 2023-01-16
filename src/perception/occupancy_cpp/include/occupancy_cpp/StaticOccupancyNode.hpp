/*
 * Package:   occupancy_cpp
 * Filename:  StaticOccupancyNode.hpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once


// Message definitions
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using nav_msgs::msg::OccupancyGrid;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;

namespace navigator
{
  namespace perception
  {

    class StaticOccupancyNode : public rclcpp::Node
    {
    public:
      StaticOccupancyNode();
      virtual ~StaticOccupancyNode();

    private:

      // Publishers
      rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub;

      // Subscribers
      rclcpp::Subscription<Clock>::SharedPtr clock_sub;
      rclcpp::Subscription<PointCloud2>::SharedPtr pcd_sub;

      // Callbacks
      void pointCloudCb(PointCloud2::SharedPtr msg);

      // Timers
      // rclcpp::TimerBase::SharedPtr map_marker_timer;

      Clock clock;

      pcl::PointCloud<pcl::PointXYZI> createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> cloud);
      
    };

  }
}
