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

class Hubble : public rclcpp::Node
{
  public:
    Hubble()
    : Node("hubble_exe")
    {
      gps_sub_ = this->create_subscription<Odometry>(
      "/gps/odometry", 10, std::bind(&Hubble::gps_cb_, this, _1));
      lidar_sub_ = this->create_subscription<PointCloud2>(
      "/lidar_front/points_raw", 10, std::bind(&Hubble::lidar_cb_, this, _1));
      
      this->declare_parameter("map_tf");
      this->get_parameter("map_tf", map_tf);
    }

  private:
    void gps_cb_(const Odometry::SharedPtr msg) const
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: "<< msg->pose.pose.position.x);
    }

    void lidar_cb_(const PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: "<< map_tf[0]);
      cached_pc = *msg;
    }

    PointCloud2 transform_using_gps(PointCloud2 inputCloud, Odometry gps) {

    }

    std::map<std::string, double> getDefaultMapTf() {
      std::map<std::string, double> default_map_tf;
      default_map_tf.insert(std::pair<std::string, double>("tx", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("ty", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("tz", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("qw", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("qx", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("qy", 0.0));
      default_map_tf.insert(std::pair<std::string, double>("qz", 0.0));
      return default_map_tf;
    }

    rclcpp::Subscription<Odometry>::SharedPtr gps_sub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub_;


    PointCloud2 cached_pc;
    std::vector<double> map_tf;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hubble>());
  rclcpp::shutdown();
  return 0;
}
