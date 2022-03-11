/*
 * Package:   nova_motion_control
 * Filename:  MotionControlNode.cpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_motion_control/MotionControlNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>
#include <string>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::MotionControl;


MotionControlNode::MotionControlNode() : rclcpp::Node("pure_pursuit_controller") {  

  this->steering_controller = std::make_unique
    <Nova::PurePursuit::PurePursuit>(1.0); // User-defined, default lookahead

  this->speed_controller = std::make_unique
    <Nova::PidController::PidController>(1.0, 1.0, 1.0, 1.0);
  
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&MotionControlNode::send_message, this));

  this->steering_control_publisher = this->create_publisher
    <SteeringPosition>("/command/steering_position", 10);

  //TODO: Interface with PidController & publish Peddle messages
  
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("reference_trajectory", 8, std::bind(&MotionControlNode::update_trajectory, this, _1));

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&MotionControlNode::update_current_position, this, _1));
  
  this->marker_array_publisher = this->create_publisher
      <MarkerArray>("/controller/trajectory_visuals", 10);
  
}

MotionControlNode::~MotionControlNode() {}

void MotionControlNode::send_message() {

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

void MotionControlNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received!");
  this->trajectory = *ptr;
}

void MotionControlNode::update_current_position(Odometry::SharedPtr ptr) {

  //RCLCPP_INFO(this->get_logger(), "Current position received!");
  this->current_position.x = ptr->pose.pose.position.x;
  this->current_position.y = ptr->pose.pose.position.y;

  visualize_markers(ptr->header.frame_id, now());
}

size_t MotionControlNode::find_closest_point() {

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

void MotionControlNode::trim_trajectory(size_t closest_point_idx) {

  if (this->trajectory.points.empty()) {
    return;
  }

  std::vector<TrajectoryPoint>::iterator delete_start, delete_end;
  delete_start = this->trajectory.points.begin();
  delete_end = this->trajectory.points.begin();
  std::advance(delete_end, closest_point_idx);

  this->trajectory.points.erase(delete_start, delete_end); // Trim the trajectory points behind the closest point
  this->trajectory.points.shrink_to_fit();                 // Release allocated memory
}

size_t MotionControlNode::find_lookahead_point(float lookahead_distance, TrajectoryPoint& current_position) {

  size_t next_waypoint_idx;

  for(size_t i = 0; i < trajectory.points.size(); i++) {
    
    auto waypoint = trajectory.points[i];
    double distance = sqrt(pow(current_position.x - waypoint.x, 2) + pow(current_position.y - waypoint.y, 2));
    
    // First waypoint to be greater than lookahead distance
    if (distance > lookahead_distance) {
      next_waypoint_idx = i;
      break;
    }

  }

  return next_waypoint_idx;
}

/** Interpolate lookahead point for steering angle calculation */
bool MotionControlNode::compute_lookahead_point() {

  if (this->trajectory.points.empty()) {
    return false;
  }

  float lookahead_distance = steering_controller->get_lookahead_distance();
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
    this->steering_controller->set_lookahead_point(p_D.getX(), p_D.getY());
  } else {

    // two intersections, take intersection in front of vehicle
    double s = sqrt(pow(lookahead_distance, 2) - pow(dist_CD, 2));
    tf2::Vector3 p_E = p_D + s * AB.normalized();
    tf2::Vector3 p_F = p_D - s * AB.normalized();

    if ((p_B - p_E).length2() < AB.length2()) {
      this->steering_controller->set_lookahead_point(p_E.getX(), p_E.getY());

    } else if ((p_B - p_F).length2() < AB.length2()) {
      this->steering_controller->set_lookahead_point(p_F.getX(), p_F.getY());

    }

  }

  return true;
}

float MotionControlNode::get_steering_angle() {
    
    if(this->trajectory.points.empty()) {
      // Error occurred, return old angle
      return this->steering_controller->get_steering_angle();
    }
    
    this->steering_controller->compute_curvature();
    return this->steering_controller->compute_steering_angle();
}

void MotionControlNode::visualize_markers(std::string frame_id, rclcpp::Time time) {

  MarkerArray recorded_markers {};

  Marker line_strip;
  line_strip.type = Marker::LINE_STRIP;
  line_strip.header.stamp = time;
  line_strip.header.frame_id = frame_id;
  line_strip.ns = "trajectory";
  line_strip.action = Marker::ADD;

  line_strip.scale.x = 0.3;
  line_strip.frame_locked = true;

  line_strip.color.r = 0.2588;
  line_strip.color.g = 0.8784;
  line_strip.color.b = 0.7725;
  line_strip.color.a = 1.0;

  for(size_t i = 0; i < this->trajectory.points.size(); i++) {

    std::shared_ptr<TrajectoryPoint> point_ptr {new TrajectoryPoint};
    *point_ptr = this->trajectory.points[i];

    geometry_msgs::msg::Point p;
    p.x = point_ptr->x;
    p.y = point_ptr->y;
    p.z = point_ptr->z;

    line_strip.points.push_back(p);
  }


  recorded_markers.markers.push_back(line_strip);
  this->marker_array_publisher->publish(recorded_markers);
  
  recorded_markers.markers.clear();
  recorded_markers.markers.shrink_to_fit();
}