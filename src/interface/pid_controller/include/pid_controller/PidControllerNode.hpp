/*
 * Package:   pid_controller
 * Filename:  PidControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "pid_controller/PidController.hpp"

using namespace std::chrono_literals;

namespace Voltron {
namespace PidController {

class PidControllerNode : public rclcpp::Node {
public:
  PidControllerNode();
  virtual ~PidControllerNode();

private:
  void update_target(const std_msgs::msg::Float32::SharedPtr command);
  void update_measurement(const std_msgs::msg::Float32::SharedPtr measurement);
  void recalculate_output();

  std::unique_ptr<PidController> controller;

  typedef std::chrono::steady_clock clock_t;
  clock_t clock;
  clock_t::time_point last_update_time;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_control_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::
    SharedPtr command_subscription;
  rclcpp::Subscription<std_msgs::msg::Float32>::
    SharedPtr measurement_subscription;
};
}
}
