/*
 * Package:   epas_translator
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

#include "nova_msgs/msg/can_frame.hpp"

namespace Voltron {
namespace EpasSteering {

class ReporterNode : public rclcpp::Node {
public:
  ReporterNode();
  ReporterNode(float epas_min, float epas_max);
  virtual ~ReporterNode();

private:
  void initialize();
  void process_frame(const nova_msgs::msg::CanFrame::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher;
  rclcpp::Subscription<nova_msgs::msg::CanFrame>::SharedPtr can_subscription;

  float epas_min;
  float epas_max;
};

}
}
