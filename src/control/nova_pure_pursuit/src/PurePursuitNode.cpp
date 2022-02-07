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
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


using namespace Nova::PurePursuit;

PurePursuitNode::PurePursuitNode() : rclcpp::Node("pure_pursuit_controller") {  
    
  this->steering_control_publisher = this->create_publisher<voltron_msgs::msg::SteeringAngle>("steering_angle", 10);
  this->trajectory_subscription = this->create_subscription<voltron_msgs::msg::Trajectories>(
    "outgoing_trajectories", 8, std::bind(&PurePursuitNode::update_trajectory, this, _1));
    
  this->controller = std::make_unique<PurePursuit>(0.1); // User-defined, default lookahead
  this->control_timer = this->create_wall_timer(message_frequency, std::bind(&PurePursuitNode::send_message, this));
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::send_message() {  
  // calculate steering angle
  auto angle = controller->get_steering_angle(trajectory);

  // format of published message
  auto steering_angle_msg = voltron_msgs::msg::SteeringAngle();
  steering_angle_msg.steering_angle = angle;

  // logging & publishing
  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", angle);
  steering_control_publisher->publish(steering_angle_msg);
}

void PurePursuitNode::update_trajectory(voltron_msgs::msg::Trajectories::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received");
  auto trajectories = ptr->trajectories;

  if(trajectories.size() == 0) {
    return; // keep using old trajectory
  }

  for(size_t i = 0; i < trajectories.size(); i++) {
    if(trajectories[i].selected == 1) {
      this->trajectory = trajectories[i];
      break;
    }
  }

}


