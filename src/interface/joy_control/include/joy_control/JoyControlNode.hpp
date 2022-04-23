/*
 * Package:   joy_control
 * Filename:  include/joy_control/JoyControlNode
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "joy_control/joy_control_params.hpp"

namespace navigator {
namespace joy_control {

class JoyControlNode final : public rclcpp::Node {
public:
  JoyControlNode();
  JoyControlNode(joy_control_params params);

private:
  void update(sensor_msgs::msg::Joy::SharedPtr message);
  void init();

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;

  bool throttle_init = false;
  bool brake_init = false;

  joy_control_params params;
};

}
}
