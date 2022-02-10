/*
 * Package:   msg_translation
 * Filename:  include/msg_translation/ControlCommandToFloatNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"

namespace navigator {
namespace msg_translation {

class ControlCommandToFloatNode : public rclcpp::Node {
public:
  ControlCommandToFloatNode();
  virtual ~ControlCommandToFloatNode();

private:
  void process_msg(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr command_subscription;
};

}
}
