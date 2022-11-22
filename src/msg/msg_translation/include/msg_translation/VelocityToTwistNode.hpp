/*
 * Package:   msg_translation
 * Filename:  include/msg_translation/VelocityToTwistNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

namespace navigator {
namespace msg_translation {

class VelocityToTwistNode : public rclcpp::Node {
public:
  VelocityToTwistNode();
  virtual ~VelocityToTwistNode();

private:
  void process_msg(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovariance>::SharedPtr twist_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_subscription;
};

}
}
