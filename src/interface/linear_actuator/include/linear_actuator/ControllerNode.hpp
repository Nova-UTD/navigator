/*
 * Package:   linear_actuator
 * Filename:  include/linear_actuator/ControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <chrono> // Time literals
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/can_frame.hpp"
#include "std_msgs/msg/float32.hpp"

#include "linear_actuator/controller_params.hpp"
#include "linear_actuator/types.hpp"

using namespace std::chrono_literals;

namespace navigator {
namespace linear_actuator {

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ControllerNode(controller_params params);
  virtual ~ControllerNode();

private:
  void send_control_message();
  void update_target_position(const std_msgs::msg::Float32::SharedPtr position);
  void init();

  float target_position;
  controller_params params;
  
  rclcpp::Publisher<voltron_msgs::msg::CanFrame>::SharedPtr can_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_subscription;
  rclcpp::TimerBase::SharedPtr control_timer;
};
}
}
