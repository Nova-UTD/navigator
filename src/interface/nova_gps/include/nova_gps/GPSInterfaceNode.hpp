/*
 * Package:   nova_gps
 * Filename:  GPSInterfaceNode.hpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include "GPSInterface.hpp"
#include <nav_msgs/msg/odometry.hpp> // Pub GPS data as odometry
#include <math.h>

namespace Nova {
namespace GPS {

class GPSInterfaceNode : public rclcpp::Node {
public:
  GPSInterfaceNode(const std::string & interface_name);
  virtual ~GPSInterfaceNode();

private:
  void send_pose();
  
  std::unique_ptr<Nova::GPS::GPSInterface> gps_interface;
  rclcpp::TimerBase::SharedPtr pose_timer;
  // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
};

}
}
