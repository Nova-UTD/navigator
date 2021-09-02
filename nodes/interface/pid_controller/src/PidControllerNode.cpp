/*
 * Package:   volron_steering_pid
 * Filename:  PidControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include "voltron_steering_pid/PidControllerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"

#include <memory>
#include <utility>
#include <time.h>

using namespace Voltron::SteeringPid;

const float max_steering_angle_radians = 0.35;

PidControllerNode::PidControllerNode(std::unique_ptr<PidController> controller)
  : rclcpp::Node("steering_pid_controller") {
  this->controller = std::move(controller);
  this->steering_control_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("steering_power", 8);
  this->command_subscription = this->create_subscription
    <autoware_auto_msgs::msg::VehicleControlCommand>("vehicle_commands", 8,
    std::bind(& PidControllerNode::update_target, this, std::placeholders::_1));
  this->measurement_subscription = this->create_subscription
    <std_msgs::msg::Float32>("real_steering_angle", 8,
    std::bind(& PidControllerNode::update_measurement, this, std::placeholders::_1));
  this->last_update_time = this->clock.now();
}

PidControllerNode::~PidControllerNode() {}

void PidControllerNode::update_target(
  const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr command) {
  float target = command->front_wheel_angle_rad;
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
