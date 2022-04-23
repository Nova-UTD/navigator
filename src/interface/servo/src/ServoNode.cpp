/*
 * Package:   servo
 * Filename:  src/ServoNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <iomanip>
#include "servo/ServoNode.hpp"
using namespace navigator::servo;

ServoNode::ServoNode() : rclcpp::Node("servo") {
  this->declare_parameter("prefix");
  this->params.prefix = this->get_parameter("prefix").as_string();
  this->declare_parameter("min");
  this->params.min = this->get_parameter("min").as_int();
  this->declare_parameter("max");
  this->params.max = this->get_parameter("max").as_int();
  this->init();
}

ServoNode::ServoNode(servo_params params) : rclcpp::Node("servo") {
  this->params = std::move(params);
  this->init();
}

void ServoNode::init() {
  this->publisher = this->create_publisher<std_msgs::msg::String>
    ("servo_commands", 8);
  this->subscription = this->create_subscription<std_msgs::msg::Float32>
    ("servo_positions", 8, std::bind
     (&ServoNode::new_position, this, std::placeholders::_1));
}

void ServoNode::new_position(std_msgs::msg::Float32::SharedPtr position) {
  int as_integer = std::lerp
    (this->params.min, this->params.max, std::clamp(position->data, 0.0f, 1.0f));
  std::stringstream message_data;
  message_data << this->params.prefix;
  message_data << std::hex;
  message_data << std::setfill('0');
  message_data << std::setw(2);
  message_data << as_integer;
  std_msgs::msg::String message;
  message.data = message_data.str();
  this->publisher->publish(message);
}
