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
#include "nova_msgs/msg/can_frame.hpp"
#include "std_msgs/msg/float32.hpp"

#include "linear_actuator/ControllerNode.hpp"

using namespace navigator::linear_actuator;

ControllerNode::ControllerNode() : Node("linear_actuator") {
  this->declare_parameter("engaged_position");
  this->declare_parameter("disengaged_position");
  this->declare_parameter("command_id");
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
  this->can_publisher = this->create_publisher<nova_msgs::msg::CanFrame>
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

  auto message = nova_msgs::msg::CanFrame();
  message.identifier = this->params.command_id;
  message.data = 0x00000000C0000A0F;
  message.data = message.data | target_position << 16;
  
  this->can_publisher->publish(message);
}

void ControllerNode::update_target_position(const std_msgs::msg::Float32::SharedPtr message) {
  this->target_position = message->data;
}
