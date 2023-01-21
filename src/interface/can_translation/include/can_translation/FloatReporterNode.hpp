/*
 * Package:   can_translation
 * Filename:  include/can_translation/FloatReporterNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "can_translation/float_reporter_params.hpp"
#include "nova_msgs/msg/can_frame.hpp"

namespace navigator {
namespace can_translation {

class FloatReporterNode : public rclcpp::Node {
public:
  FloatReporterNode();
  FloatReporterNode(float_reporter_params params);
  virtual ~FloatReporterNode();

private:
  void process_frame(const nova_msgs::msg::CanFrame::SharedPtr msg);
  void init();

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr result_publisher;
  rclcpp::Subscription<nova_msgs::msg::CanFrame>::SharedPtr can_subscription;

  float_reporter_params params;
};

}
}
