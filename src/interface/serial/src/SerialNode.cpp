/*
 * Package:   serial
 * Filename:  src/SerialNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <chrono>
#include <sstream>
#include "serial/SerialNode.hpp"
using namespace navigator::serial;
using namespace std::chrono_literals;

SerialNode::SerialNode() : rclcpp::Node("gps") {
  this->declare_parameter("device_name");
  this->params.device_name = this->get_parameter("device_name").as_string();
  this->init();
}

SerialNode::SerialNode(serial_params params) : rclcpp::Node("serial") {
  this->params = std::move(params);
  this->init();
}

void SerialNode::init() {
  this->serial_port = std::make_unique<SerialPort>(this->params.device_name);
  this->timer = this->create_wall_timer
    (0.25s, std::bind(&SerialNode::check_messages, this));
  this->publisher = this->create_publisher<std_msgs::msg::String>
    ("incoming_lines", 8);
  this->subscription = this->create_subscription<std_msgs::msg::String>
    ("outgoing_lines", 8, std::bind
     (&SerialNode::send_message, this, std::placeholders::_1));
}

void SerialNode::check_messages() {
  std::optional<std::string> line;
  line = this->serial_port->get_line();
  while(line.has_value()) {
    std_msgs::msg::String message;
    message.data = line.value();
    this->publisher->publish(message);
    line = this->serial_port->get_line();
  }
}

void SerialNode::send_message(std_msgs::msg::String::SharedPtr message) {
  this->serial_port->send(message->data);
}
