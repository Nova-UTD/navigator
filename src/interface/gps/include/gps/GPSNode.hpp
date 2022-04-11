/*
 * Package:   gps
 * Filename:  include/gps/GPSNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "gps/gps_params.hpp"
#include "gps/SerialPort.hpp"

namespace navigator {
namespace gps {

class GPSNode final : public rclcpp::Node {
public:
  GPSNode();
  GPSNode(gps_params params);

private:
  void check_messages();
  void init();

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
  std::unique_ptr<SerialPort> gps_port;
  gps_params params;
};

}
}
