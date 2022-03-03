/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.cpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_pure_pursuit/PurePursuitNode.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


using namespace Nova::PurePursuit;

PurePursuitNode::PurePursuitNode() : rclcpp::Node("pure_pursuit_controller") {  

  this->controller = std::make_unique
    <PurePursuit>(0.1); // User-defined, default lookahead
  
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&PurePursuitNode::send_message, this));

  this->steering_control_publisher = this->create_publisher
    <SteeringPosition>("steering_angle", 10);
  
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("reference_trajectory", 8, std::bind(&PurePursuitNode::update_trajectory, this, _1));
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::send_message() {

  /* TODO:
  - Select lookahead & closest points using util functions
  - set new points for the controller
  - This must be done regardless of whether having received a new trajectory 
  */

  auto angle = controller->get_steering_angle(trajectory);

  // format of published message
  auto steering_angle_msg = SteeringPosition();
  steering_angle_msg.data = angle;

  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", angle);
  steering_control_publisher->publish(steering_angle_msg);
}

void PurePursuitNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received!");
  this->trajectory = *ptr;
}