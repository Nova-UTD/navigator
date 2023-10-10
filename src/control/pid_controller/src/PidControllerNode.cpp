/*
 * Package:   pid_controller
 * Filename:  PidControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include "pid_controller/PidControllerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <memory>
#include <utility>
#include <time.h>

using namespace Voltron::PidController;

const float max_steering_angle_radians = 0.35;

PidControllerNode::PidControllerNode() : rclcpp::Node("pid_controller") {
  this->declare_parameter("KP");
  this->declare_parameter("KI");
  this->declare_parameter("KD");
  this->declare_parameter("time_delta_cap_seconds");
  float KP = this->get_parameter("KP").as_double();
  float KI = this->get_parameter("KI").as_double();
  float KD = this->get_parameter("KD").as_double();
  float time_delta_cap_seconds = this->get_parameter("time_delta_cap_seconds").as_double();
  this->controller = std::make_unique
    <PidController>(KP, KI, KD, time_delta_cap_seconds);
  this->steering_control_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("output", 8);
  this->command_subscription = this->create_subscription
    <std_msgs::msg::Float32>("target", 8,
    std::bind(& PidControllerNode::update_target, this, std::placeholders::_1));
  this->measurement_subscription = this->create_subscription
    <std_msgs::msg::Float32>("measurement", 8,
    std::bind(& PidControllerNode::update_measurement, this, std::placeholders::_1));
  this->last_update_time = this->clock.now();
}

PidControllerNode::~PidControllerNode() {}

void PidControllerNode::update_target(
  const std_msgs::msg::Float32::SharedPtr command) {
  float target = command->data;
  if(target > max_steering_angle_radians) target = max_steering_angle_radians;
  if(target < (-1 * max_steering_angle_radians)) target = (-1 * max_steering_angle_radians);
  this->controller->set_target(target);
  this->recalculate_output();
}

void PidControllerNode::update_measurement(
  const std_msgs::msg::Float32::SharedPtr measurement) {
  this->controller->set_measurement(measurement->data);
  this->recalculate_output();
}

void PidControllerNode::recalculate_output() {
  clock_t::time_point now = this->clock.now();
  clock_t::duration time_delta = now - this->last_update_time;
  this->last_update_time = now;
  float time_delta_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(time_delta)
    .count(); // Convert to seconds float
  float steering_power = this->controller->compute(time_delta_seconds);
  auto message = std_msgs::msg::Float32();
  message.data = steering_power;
  this->steering_control_publisher->publish(message);
}
