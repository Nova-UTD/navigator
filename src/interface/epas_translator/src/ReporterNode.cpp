/*
 * Package:   epas_translator
 * Filename:  ReporterNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "nova_msgs/msg/can_frame.hpp"

#include "epas_translator/ReporterNode.hpp"

using namespace Voltron::EpasSteering;

typedef uint16_t can_id_t;
typedef uint64_t can_data_t;

constexpr can_id_t epas_can_message_2_identifier = 0x292;
constexpr int angle_byte_index = 0;
constexpr int angle_bit_shift = angle_byte_index * 8;
constexpr can_data_t angle_mask = 0xFF << (8 * angle_bit_shift);

// TODO Make this an actual parameter
// Maximum angle we can steer to on either side, in radians
const float steering_angle_max = 0.58294;

ReporterNode::ReporterNode() : rclcpp::Node("steering_reporter") {
  this->declare_parameter("is_calibrated");
  if(! this->get_parameter("is_calibrated").as_bool()) {
    throw std::runtime_error("EPAS ECU stops not calibrated!");
  }

  this->declare_parameter("epas_min");
  this->declare_parameter("epas_max");
  this->epas_min = this->get_parameter("epas_min").as_double();
  this->epas_max = this->get_parameter("epas_max").as_double();

  this->initialize();
}

ReporterNode::ReporterNode(float epas_min, float epas_max)
  : rclcpp::Node("steering_reporter") {

  this->epas_min = epas_min;
  this->epas_max = epas_max;
  this->initialize();
}

ReporterNode::~ReporterNode() {}

void ReporterNode::process_frame(const nova_msgs::msg::CanFrame::SharedPtr incoming_frame) {
  if(incoming_frame->identifier != epas_can_message_2_identifier) return;

  float current_angle = (incoming_frame->data & angle_mask) >> angle_bit_shift;
  current_angle = (current_angle - this->epas_min) /
    (this->epas_max - this->epas_min); // Value from 0 to 1
  current_angle = (current_angle * 2) - 1; // Value from -1 to 1

  auto message = std_msgs::msg::Float32();
  message.data = current_angle * steering_angle_max; // Angle in radians
  this->angle_publisher->publish(message);
}

void ReporterNode::initialize() {
  this->angle_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("epas_translator_real_steering_angle", 8);
  this->can_subscription = this->create_subscription<nova_msgs::msg::CanFrame>
    ("epas_translator_incoming_can_frames", 8,
     std::bind(& ReporterNode::process_frame, this, std::placeholders::_1));
}
