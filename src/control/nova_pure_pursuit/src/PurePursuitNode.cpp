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
    <SteeringAngle>("steering_angle", 10);
  
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("reference_trajectory", 8, std::bind(&PurePursuitNode::update_trajectory, this, _1));
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::send_message() {

  // Temporary
  TrajectoryPoint current_point;
  current_point.x = 0.0;
  current_point.y = 0.0;

  // For later use: obtain the new target longitudinal velocity
  TrajectoryPoint closest_point = compute_closest_point(current_point);

  // TODO: Select & update lookahead_point of controller
  auto angle = controller->get_steering_angle(trajectory);

  // format of published message
  auto steering_angle_msg = SteeringAngle();
  steering_angle_msg.steering_angle = angle;

  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", angle);
  steering_control_publisher->publish(steering_angle_msg);

}

void PurePursuitNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received!");
  this->trajectory = *ptr;
}

TrajectoryPoint PurePursuitNode::compute_closest_point(TrajectoryPoint current_point) {

  TrajectoryPoint closest_point;
  float minimum_distance = std::numeric_limits<float>::max();

  for (size_t i = 0; i < this->trajectory.points.size(); i++) {

    std::shared_ptr<TrajectoryPoint> point_ptr {new TrajectoryPoint};
    *point_ptr = this->trajectory.points[i];

    float x_diff = point_ptr->x - current_point.x;
    float y_diff = point_ptr->y - current_point.y;
    float distance = sqrt( (x_diff*x_diff) + (y_diff*y_diff) );

    if (distance < minimum_distance) {
      minimum_distance = distance;
      closest_point = *point_ptr;
    }
  }

  return closest_point;
}