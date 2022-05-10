/*
 * Package:   vehicle_kinematics
 * Filename:  VehicleKinematicsNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <functional>
#include <cmath>

#include "vehicle_kinematics/VehicleKinematicsNode.hpp"

using navigator::vehicle_kinematics::VehicleKinematicsNode;

VehicleKinematicsNode::VehicleKinematicsNode() : Node("vehicle_kinematics") {
  this->declare_parameter("vehicle_length");
  this->vehicle_length = this->get_parameter("vehicle_length").as_double();
  this->declare_parameter("speed_covariance");
  this->speed_covariance = this->get_parameter("speed_covariance").as_double();
  this->declare_parameter("angle_covariance");
  this->angle_covariance = this->get_parameter("angle_covariance").as_double();
  this->twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>
    ("vehicle_kinematics_twist", 8);
  this->speed_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("vehicle_kinematics_speed", 8,
     bind(& VehicleKinematicsNode::update_speed, this, std::placeholders::_1));
  this->angle_subscription = this->create_subscription<std_msgs::msg::Float32>
    ("vehicle_kinematics_angle", 8,
     bind(& VehicleKinematicsNode::update_angle, this, std::placeholders::_1));
}

void VehicleKinematicsNode::send_message() {
  float angular_velocity = this->speed * tan(this->angle) / this->vehicle_length;
  auto message = geometry_msgs::msg::TwistWithCovarianceStamped();
  message.header.frame_id = "base_link";
  message.header.stamp = this->get_clock()->now();
  message.twist.twist.linear.x = this->speed;
  message.twist.twist.angular.z = angular_velocity;
  message.twist.covariance[0] = 0.25;
  message.twist.covariance[35] = 1;
  this->twist_publisher->publish(message);
}

void VehicleKinematicsNode::update_speed(const std_msgs::msg::Float32::SharedPtr message) {
  this->speed = message->data;
  this->send_message();
}

void VehicleKinematicsNode::update_angle(const std_msgs::msg::Float32::SharedPtr message) {
  this->angle = message->data;
  this->send_message();
}
