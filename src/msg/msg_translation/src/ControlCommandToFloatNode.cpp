/*
 * Package:   msg_translation
 * Filename:  src/VelocityToTwistNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"

#include "msg_translation/ControlCommandToFloatNode.hpp"

using namespace navigator::msg_translation;

ControlCommandToFloatNode::ControlCommandToFloatNode() : rclcpp::Node("control_command_to_float") {
  this->angle_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("angle_topic", 8);
  this->command_subscription = this->create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>
    ("command_topic", 8,
     std::bind(& ControlCommandToFloatNode::process_msg, this, std::placeholders::_1));
}


ControlCommandToFloatNode::~ControlCommandToFloatNode() {}

void ControlCommandToFloatNode::process_msg(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr msg) {
  auto message = std_msgs::msg::Float32(); // Send the message
  message.data = msg->front_wheel_angle_rad;
  this->angle_publisher->publish(message);
}
  
