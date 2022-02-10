/*
 * Package:   linear_actuator
 * Filename:  src/ControllerNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <cmath>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/can_frame.hpp"
#include "std_msgs/msg/float32.hpp"

#include "linear_actuator/ControllerNode.hpp"

using namespace navigator::linear_actuator;

ControllerNode::ControllerNode() : Node("linear_actuator") {
  this->get_parameter("engaged_position", this->params.engaged_position);
  this->get_parameter("disengaged_position", this->params.disengaged_position);
  this->get_parameter("command_id", this->params.command_id);
  this->init();
}

ControllerNode::ControllerNode(controller_params params) : Node("linear_actuator") {
  this->params = params;
  this->init();
}

ControllerNode::~ControllerNode() {}

void ControllerNode::init() {
  this->can_publisher = this->create_publisher<voltron_msgs::msg::CanFrame>
    ("outgoing_can_frames", 8);
  this->position_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("target_position", 8, bind(& ControllerNode::update_target_position, this, std::placeholders::_1));
  this->control_timer = this->create_wall_timer(100ms,
    bind(& ControllerNode::send_control_message, this));
}  

void ControllerNode::send_control_message() {
  can_data_t target_position = (can_data_t) std::lerp(
      (float) this->params.disengaged_position,
      (float) this->params.engaged_position,
      this->target_position);
  can_data_t target_position_low_bit = target_position % 256;
  can_data_t target_position_high_bit = target_position / 256;

  auto message = voltron_msgs::msg::CanFrame();
  message.identifier = this->params.command_id;
  message.data = 0x0F0A00C000000000;
  message.data = message.data | target_position_low_bit << 40;
  message.data = message.data | target_position_high_bit << 32;
  
  this->can_publisher->publish(message);
}

void ControllerNode::update_target_position(const std_msgs::msg::Float32::SharedPtr message) {
  this->target_position = message->data;
}
