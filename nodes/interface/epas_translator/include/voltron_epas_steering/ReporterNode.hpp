/*
 * Package:   volron_epas_steering
 * Filename:  ReporterNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "voltron_can/msg/can_frame.hpp"

namespace Voltron {
namespace EpasSteering {

class ReporterNode : public rclcpp::Node {
public:
  ReporterNode(const std::string & interface_name, float epas_min, float epas_max);
  virtual ~ReporterNode();

private:
  void process_frame(const voltron_can::msg::CanFrame::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher;
  rclcpp::Subscription<voltron_can::msg::CanFrame>::SharedPtr can_subscription;

  float epas_min;
  float epas_max;
};
  
}
}