/*
 * Package:   gps
 * Filename:  src/GPSNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <chrono>
#include <sstream>
#include "gps/GPSNode.hpp"
#include "gps/GPSFix.hpp"
using namespace navigator::gps;
using namespace std::chrono_literals;

GPSNode::GPSNode() : rclcpp::Node("gps") {
  this->declare_parameter("device_name");
  this->params.device_name = this->get_parameter("device_name").as_string();
  this->init();
}

GPSNode::GPSNode(gps_params params) : rclcpp::Node("gps") {
  this->params = std::move(params);
  this->init();
}

void GPSNode::init() {
  this->gps_port = std::make_unique<SerialPort>(this->params.device_name);
  this->timer = this->create_wall_timer(0.25s, std::bind(&GPSNode::check_messages, this));
  this->odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("gps_odometry_topic", 8);
}

void GPSNode::check_messages() {
  std::optional<std::string> line;
  std::optional<std::string> next_line;
  next_line = this->gps_port->get_line();
  while(next_line.has_value()) {
    line = next_line;
    next_line = this->gps_port->get_line();
  }
  if(line.has_value()) {
    GPSFix fix(line.value());
    if(!fix.valid()) {
      RCLCPP_INFO(this->get_logger(), "No GPS fix!");
    } else {
      this->odometry_publisher->publish(fix.to_message());
    }
  }
}
