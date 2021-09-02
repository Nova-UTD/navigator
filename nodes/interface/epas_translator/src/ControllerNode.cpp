/*
 * Package:   volron_epas_steering
 * Filename:  ControllerNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <algorithm> // std::clamp
#include <cstdint> // Fixed-width integers
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/can_frame.hpp"
#include "std_msgs/msg/float32.hpp"

#include "voltron_epas_steering/ControllerNode.hpp"

using namespace Voltron::EpasSteering;

can_id_t can_message_3_identifier = 0x296;

ControllerNode::ControllerNode(const std::string & interface_name) :
  Node("steering_controller_" + interface_name) {

  this->can_publisher = this->create_publisher<voltron_msgs::msg::CanFrame>
    ("outgoing_can_frames_" + interface_name, 8);
  this->power_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("steering_power", 8, bind(& ControllerNode::update_power, this, std::placeholders::_1));
  this->control_timer = this->create_wall_timer(control_message_frequency,
    bind(& ControllerNode::send_control_message, this));
}

ControllerNode::~ControllerNode() {}

void ControllerNode::send_control_message() {
  auto message = voltron_msgs::msg::CanFrame();
  message.identifier = can_message_3_identifier;
  message.data = (255 - this->power);
  message.data <<= 8;
  message.data += this->power;
  message.data <<= 8;
  message.data += steering_map;
  this->can_publisher->publish(message);
}

void ControllerNode::update_power(const std_msgs::msg::Float32::SharedPtr message) {
  float f_power = (message->data * 128) + 127;
  if(f_power < 0) f_power = 0;
  if(f_power > 255) f_power = 255;
  this->power = (uint8_t) f_power;
}
