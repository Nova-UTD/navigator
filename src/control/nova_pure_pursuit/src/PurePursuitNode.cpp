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
#include <iostream>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::PurePursuit;


PurePursuitNode::PurePursuitNode() : rclcpp::Node("pure_pursuit_controller") {  

  this->controller = std::make_unique
    <PurePursuit>(0.1); // User-defined, default lookahead
  
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&PurePursuitNode::send_message, this));

  this->steering_control_publisher = this->create_publisher
    <SteeringPosition>("/command/steering_position", 10);
  
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("reference_trajectory", 8, std::bind(&PurePursuitNode::update_trajectory, this, _1));

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&PurePursuitNode::update_current_position, this, _1));
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::send_message() {

  // Report error & delete trajectory points behind the closest point
  size_t closest_point_idx = find_closest_point();
  trim_trajectory(closest_point_idx);

  compute_lookahead_point();

  // format of published message
  auto steering_angle_msg = SteeringPosition();
  steering_angle_msg.data = get_steering_angle();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", steering_angle_msg);
  steering_control_publisher->publish(steering_angle_msg);
}

void PurePursuitNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received!");
  this->trajectory = *ptr;
}

void PurePursuitNode::update_current_position(Odometry::SharedPtr ptr) {

  //RCLCPP_INFO(this->get_logger(), "Current position received!");
  this->current_position.x = ptr->pose.pose.position.x;
  this->current_position.y = ptr->pose.pose.position.y;
}

size_t PurePursuitNode::find_closest_point() {

  size_t closest_point_idx = 0;
  TrajectoryPoint current_position = this->current_position;

  float distance = 0.0;
  float minimum_distance = std::numeric_limits<float>::max();

  for (size_t i = 0; i < this->trajectory.points.size(); i++) {

    std::shared_ptr<TrajectoryPoint> point_ptr {new TrajectoryPoint};
    *point_ptr = this->trajectory.points[i];

    distance = sqrt(pow(current_position.x - point_ptr->x, 2) + pow(current_position.y - point_ptr->y, 2));

    if (distance < minimum_distance) {
      minimum_distance = distance;
      closest_point_idx = i;
    }

  }

  std::cout << "Lateral offset error: " << distance << std::endl;

  return closest_point_idx;
}

void PurePursuitNode::trim_trajectory(size_t closest_point_idx) {

  if (this->trajectory.points.empty()) {
    return;
  }

  std::vector<TrajectoryPoint>::iterator delete_start, delete_end;
  delete_start = this->trajectory.points.begin();
  delete_end = this->trajectory.points.begin();

  for (size_t i = 0; i < closest_point_idx; i++) {
    delete_end++;
  }
  
  // Trim the trajectory points behind the closest point
  this->trajectory.points.erase(delete_start, delete_end);
}

size_t PurePursuitNode::find_lookahead_point(float lookahead_distance, TrajectoryPoint current_position) {

  size_t next_waypoint_idx;

  for(size_t i = 0; i < trajectory.points.size(); i++) {
    
    auto waypoint = trajectory.points[i];
    double distance = sqrt(pow(current_position.x - waypoint.x, 2) + pow(current_position.y - waypoint.y, 2));
    
    // First waypoint to be greater than lookahead distance
    if (distance > lookahead_distance) {
      next_waypoint_idx = i;
    }

  }

  return next_waypoint_idx;
}

/** Interpolate lookahead point for steering angle calculation */
bool PurePursuitNode::compute_lookahead_point() {

  if (this->trajectory.points.empty()) {
    return false;
  }

  float lookahead_distance = controller->get_lookahead_distance();
  TrajectoryPoint current_position = this->current_position;


  // Find first point outside lookahead distance to use to find lookahead point
  int next_waypoint_idx = find_lookahead_point(lookahead_distance, current_position);


  // TODO perform check that curve exists
  // TODO special logic if chosen waypoint is first or last in trajectory


  const TrajectoryPoint start = trajectory.points[next_waypoint_idx - 1];
  const TrajectoryPoint end = trajectory.points[next_waypoint_idx];

  // Project vehicle's current position onto line between start and end points
  const tf2::Vector3 p_A(start.x, start.y, 0.0);
  const tf2::Vector3 p_B(end.x, end.y, 0.0);
  const tf2::Vector3 p_C(current_position.x, current_position.y, 0.0);
  const tf2::Vector3 AB = p_B - p_A;
  const tf2::Vector3 AC = p_C - p_A;
  const tf2::Vector3 p_D = p_A + AC.dot(AB) / AB.dot(AB) * AB;
  const double dist_CD = (p_D - p_C).length();

  // set lookahead point based on intersection

  if (dist_CD > lookahead_distance) {
    // circle surrounding vehicle's current location 
    // does not intersect with trajectory
    return false;
  } else if (dist_CD == lookahead_distance) {
    // lookahead circle intersects exactly at projection
    this->controller->set_lookahead_point(p_D.getX(), p_D.getY());
  } else {

    // two intersections, take intersection in front of vehicle
    double s = sqrt(pow(lookahead_distance, 2) - pow(dist_CD, 2));
    tf2::Vector3 p_E = p_D + s * AB.normalized();
    tf2::Vector3 p_F = p_D - s * AB.normalized();

    if ((p_B - p_E).length2() < AB.length2()) {
      this->controller->set_lookahead_point(p_E.getX(), p_E.getY());

    } else if ((p_B - p_F).length2() < AB.length2()) {
      this->controller->set_lookahead_point(p_F.getX(), p_F.getY());

    }

  }

  return true;
}

float PurePursuitNode::get_steering_angle() {
    
    if(this->trajectory.points.empty()) {
      // Error occurred, return old angle
      return controller->get_steering_angle();
    }
    
    this->controller->compute_curvature();
    return this->controller->compute_steering_angle();
}