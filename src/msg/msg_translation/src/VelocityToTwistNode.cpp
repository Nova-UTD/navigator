/*
 * Package:   msg_translation
 * Filename:  src/VelocityToTwistNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

#include "msg_translation/VelocityToTwistNode.hpp"

using namespace navigator::msg_translation;

VelocityToTwistNode::VelocityToTwistNode() : rclcpp::Node("velocity_to_twist") {
  this->twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovariance>
    ("twist_topic", 8);
  this->velocity_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("velocity_topic", 8,
     std::bind(& VelocityToTwistNode::process_msg, this, std::placeholders::_1));
}


VelocityToTwistNode::~VelocityToTwistNode() {}

void VelocityToTwistNode::process_msg(const std_msgs::msg::Float32::SharedPtr msg) {
  auto message = geometry_msgs::msg::TwistWithCovariance(); // Send the message
  message.twist.linear.x = msg->data;
  this->twist_publisher->publish(message);
}
  
