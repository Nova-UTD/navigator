/*
 * Package:   volron_epas_steering
 * Filename:  ReporterNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "voltron_can/msg/can_frame.hpp"
#include "voltron_can/CanFrame.hpp" // Data types

#include "voltron_epas_steering/ReporterNode.hpp"

using namespace Voltron::EpasSteering;

typedef Voltron::Can::CanFrame::identifier_t can_id_t;
typedef Voltron::Can::CanFrame::data_t can_data_t;

constexpr can_id_t epas_can_message_2_identifier = 0x292;
constexpr int angle_byte_index = 0;
constexpr int angle_bit_shift = angle_byte_index * 8;
constexpr can_data_t angle_mask = 0xFF << (8 * angle_bit_shift);

// TODO Make this an actual parameter
// Maximum angle we can steer to on either side, in radians
const float steering_angle_max = 0.58294;

ReporterNode::ReporterNode(const std::string & interface, float epas_min, float epas_max)
  : rclcpp::Node("steering_reporter_" + interface) {
  this->angle_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("real_steering_angle", 8);
  this->can_subscription = this->create_subscription<voltron_can::msg::CanFrame>
    ("incoming_can_frames_" + interface, 8,
     std::bind(& ReporterNode::process_frame, this, std::placeholders::_1));
  this->epas_min = epas_min;
  this->epas_max = epas_max;
}

ReporterNode::~ReporterNode() {}

void ReporterNode::process_frame(const voltron_can::msg::CanFrame::SharedPtr incoming_frame) {
  if(incoming_frame->identifier != epas_can_message_2_identifier) return;
  
  float current_angle = (incoming_frame->data & angle_mask) >> angle_bit_shift;
  current_angle = (current_angle - this->epas_min) /
    (this->epas_max - this->epas_min); // Value from 0 to 1
  current_angle = (current_angle * 2) - 1; // Value from -1 to 1

  auto message = std_msgs::msg::Float32();
  message.data = current_angle * steering_angle_max; // Angle in radians
  this->angle_publisher->publish(message);
}
