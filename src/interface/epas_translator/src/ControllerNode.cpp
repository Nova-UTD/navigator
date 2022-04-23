/*
 * Package:   epas_translator
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

#include "epas_translator/ControllerNode.hpp"

using namespace Voltron::EpasSteering;

can_id_t can_message_3_identifier = 0x296;

ControllerNode::ControllerNode() : Node("steering_controller") {
  this->can_publisher = this->create_publisher<voltron_msgs::msg::CanFrame>
    ("epas_translator_outgoing_can_frames", 8);
  this->power_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("epas_translator_steering_power", 8,
     bind(& ControllerNode::update_power, this, std::placeholders::_1));
  this->enable_subscription = this->create_subscription<std_msgs::msg::Bool>
    ("epas_translator_enable", 8,
     bind(& ControllerNode::enable, this, std::placeholders::_1));
  this->control_timer = this->create_wall_timer(control_message_frequency,
    bind(& ControllerNode::send_control_message, this));
  this->last_enabled = std::chrono::system_clock::now();
}

ControllerNode::~ControllerNode() {}

void ControllerNode::send_control_message() {
  if(!enabled) return;
  std::chrono::duration<double> time_since_enable
    = std::chrono::system_clock::now() - this->last_enabled;
  int ms_since_enabled = std::chrono::duration_cast
    <std::chrono::milliseconds>(time_since_enable).count();
  if(ms_since_enabled > 250) return;
  
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

void ControllerNode::enable(const std_msgs::msg::Bool::SharedPtr message) {
  this->enabled = message->data;
  this->last_enabled = std::chrono::system_clock::now();
}
