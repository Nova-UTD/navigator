/*
 * Package:   hubble
 * Filename:  hubble_main.cpp
 * Author:    Will Heitman
 * Email:     Will.Heitman@UTDallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1; // I don't know what this does. Can someone explain? WSH.

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::Point;

class GridDivider // This is deliberately not a ROS node. It's just a simple C++ utility.
{
  public:
    GridDivider()
    {
    }

  private:
    PointCloud2 cached_pc;
    std::vector<double> map_tf;
};

int main(int argc, char * argv[])
{
  return 0;
}
