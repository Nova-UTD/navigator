/*
 * Package:   vehicle_kinematics
 * Filename:  VehicleKinematicsNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace navigator {
namespace vehicle_kinematics {

class VehicleKinematicsNode final : public rclcpp::Node {
public:
  VehicleKinematicsNode();

private:
  void update_speed(const std_msgs::msg::Float32::SharedPtr message);
  void update_angle(const std_msgs::msg::Float32::SharedPtr message);
  void send_message();

  float speed = 0;
  float angle = 0;

  float vehicle_length;
  float speed_covariance;
  float angle_covariance;
  
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription;
};
}
}
