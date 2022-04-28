/*
 * Package:   joy_control
 * Filename:  src/JoyControlNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "joy_control/JoyControlNode.hpp"
using namespace navigator::joy_control;

JoyControlNode::JoyControlNode() : rclcpp::Node("joy_control") {
  this->declare_parameter("throttle_axis");
  this->params.throttle_axis = this->get_parameter("throttle_axis").as_int();
  this->declare_parameter("brake_axis");
  this->params.brake_axis = this->get_parameter("brake_axis").as_int();
  this->declare_parameter("steering_axis");
  this->params.steering_axis = this->get_parameter("steering_axis").as_int();
  this->declare_parameter("enable_button");
  this->params.enable_button = this->get_parameter("enable_button").as_int();
  this->declare_parameter("max_steering_angle");
  this->params.max_steering_angle
    = this->get_parameter("max_steering_angle").as_double();
  this->init();
}

JoyControlNode::JoyControlNode(joy_control_params params)
  : rclcpp::Node("joy_control") {
  this->params = params;
  this->init();
}

void JoyControlNode::init() {
  this->throttle_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("joy_control_throttle", 8);
  this->brake_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("joy_control_brake", 8);
  this->steering_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("joy_control_steering", 8);
  this->enable_publisher = this->create_publisher<std_msgs::msg::Bool>
    ("joy_control_enable", 8);
  this->joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>
    ("joy_control_input", 8, std::bind
     (&JoyControlNode::update, this, std::placeholders::_1));
}

void JoyControlNode::update(sensor_msgs::msg::Joy::SharedPtr state) {
  std_msgs::msg::Float32 throttle_message;
  std_msgs::msg::Float32 brake_message;
  std_msgs::msg::Float32 steering_message;
  std_msgs::msg::Bool enable_message;

  float raw_throttle = state->axes[this->params.throttle_axis];
  float raw_brake = state->axes[this->params.brake_axis];
  float raw_steering = state->axes[this->params.steering_axis];
  int raw_enable = state->buttons[this->params.enable_button];

  // Until moved, throttle and brake initially read 0
  if(raw_throttle != 0.0) this->throttle_init = true;
  if(raw_brake != 0.0) this->brake_init = true;

  bool enable = raw_enable == 1;
  enable_message.data = enable;
  
  throttle_message.data = this->throttle_init && enable ?
    (raw_throttle * -0.5) + 0.5 : 0.0;
  brake_message.data = this->brake_init && enable ?
    (raw_brake * -0.5) + 0.5 : 0.0;
  steering_message.data = enable ?
    raw_steering * this->params.max_steering_angle * -1 : 0.0;
  
  this->throttle_publisher->publish(throttle_message);
  this->brake_publisher->publish(brake_message);
  this->steering_publisher->publish(steering_message);
  this->enable_publisher->publish(enable_message);
}
