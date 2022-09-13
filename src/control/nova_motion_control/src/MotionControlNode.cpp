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
#include <cmath>
#include <time.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

const float max_steering_angle_radians = 0.35;


using namespace Nova::MotionControl;


MotionControlNode::MotionControlNode() : rclcpp::Node("pure_pursuit_controller") {  

  // Controllers
  this->steering_controller = std::make_unique
    <Nova::PurePursuit::PurePursuit>(10.0); // User-defined, default lookahead

  this->speed_controller = std::make_unique
    <Nova::PidController::PidController>(1.0, 1.0, 1.0, 1.0);
  
  // Utilities
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&MotionControlNode::send_message, this));

  this->marker_array_publisher = this->create_publisher
      <MarkerArray>("/controller/trajectory_visuals", 10);

  // Input
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("final_path", 8, std::bind(&MotionControlNode::update_trajectory, this, _1));

  this->odometry_subscription = this->create_subscription
    <Odometry>("/odom/filtered", 8, std::bind(&MotionControlNode::update_odometry, this, _1));
  
  // Output
  this->steering_control_publisher = this->create_publisher
    <SteeringPosition>("/command/steering_position", 10);

  this->throttle_control_publisher = this->create_publisher
    <PeddlePosition>("/command/throttle_position", 10);

  this->brake_control_publisher = this->create_publisher
    <PeddlePosition>("/command/brake_position", 10);
  
  //TODO: Interface with PidController & publish Peddle messages

  this->last_update_time = this->clock.now();
  
}

MotionControlNode::~MotionControlNode() {}

void MotionControlNode::send_message() {

  //---------------------Throttle/Brake------------------
  clock_t::time_point now = this->clock.now();
  clock_t::duration time_delta = now - this->last_update_time;
  this->last_update_time = now;
  float time_delta_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(time_delta).count();

  float acceleration = this->speed_controller->compute(time_delta_seconds);

  // TODO: use a ratio to scale the acceleration to 0.0 to 1.0 for the message commands

  auto throttle_position_msg = PeddlePosition();
  auto brake_position_msg = PeddlePosition();

  if (acceleration > 0) { // Speed up
    throttle_position_msg.data = 1.0;
    brake_position_msg.data = 0.0;
  } else { // Slow down
    brake_position_msg.data = 1.0;
    throttle_position_msg.data = 0.0;
  }

  // this->throttle_control_publisher->publish(throttle_position_msg);
  // this->brake_control_publisher->publish(brake_position_msg);

  //----------------------STEERING----------------------
  compute_lookahead_point();

  // format of published message
  auto steering_angle_msg = SteeringPosition();
  float steering_direction =  this->steering_direction();

  // Method 1:
  steering_angle_msg.data = abs(get_steering_angle()) * this->steering_direction();

  // Method 2:
  if (steering_direction > 0) // steer right
    steering_angle_msg.data = 3.14 / 6;
  else if (steering_direction < 0)
    steering_angle_msg.data = -3.14 / 6;

  // For the curves:
  if (this->current_speed != 0.0)
    steering_angle_msg.data *= 1 / this->current_speed;


  RCLCPP_INFO(this->get_logger(), "Publishing Steering Angle: '%f'", steering_angle_msg.data);
  steering_control_publisher->publish(steering_angle_msg);
}

void MotionControlNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received!");
  this->trajectory = *ptr;

  compute_target_speed();
}

void MotionControlNode::update_odometry(Odometry::SharedPtr ptr) {

  //RCLCPP_INFO(this->get_logger(), "Current position received!");
  this->current_position.x = ptr->pose.pose.position.x;
  this->current_position.y = ptr->pose.pose.position.y;

  tf2::Vector3 vector_velocity(ptr->twist.twist.linear.x, ptr->twist.twist.linear.y, ptr->twist.twist.linear.z);
  this->current_speed = std::sqrt( vector_velocity.dot(vector_velocity) ); // magnitude of vector
  //std::cout << "CURRENT SPEED: " << this->current_speed << std::endl;
  this->speed_controller->set_measurement(current_speed);

  float z = ptr->pose.pose.orientation.z;
  this->heading = 2 * asin(z);

  //std::cout << "Heading: " << this->heading << std::endl;

  // Delete trajectory points behind the closest point
  size_t closest_point_idx = find_closest_point();
  trim_trajectory(closest_point_idx);

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

    //distance = sqrt(pow(current_position.x - point_ptr->x, 2) + pow(current_position.y - point_ptr->y, 2));
    
    distance = this->distance(current_position, *point_ptr);

    if (distance < minimum_distance) {
      minimum_distance = distance;
      closest_point_idx = i;
    }

  }

  this->steering_controller->set_displacement_error(minimum_distance);
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

size_t MotionControlNode::find_next_waypoint(float lookahead_distance, TrajectoryPoint& current_position) {

  size_t next_waypoint_idx;

  for(size_t i = 0; i < trajectory.points.size(); i++) {
    
    auto waypoint = trajectory.points[i];
    //double distance = sqrt(pow(current_position.x - waypoint.x, 2) + pow(current_position.y - waypoint.y, 2));

    float distance = this->distance(current_position, waypoint);
    
    // First waypoint to be greater than lookahead distance
    if (distance > lookahead_distance) {
      next_waypoint_idx = i;
      break;
    }

  }

  return next_waypoint_idx;
}

void MotionControlNode::compute_target_speed() {

  if (this->trajectory.points.empty()) {
    return;
  }

  if (this->trajectory.points.size() > 1) {

    float target_speed = abs(this->trajectory.points[1].longitudinal_velocity_mps);
    this->speed_controller->set_target(target_speed);
  } else {
    this->speed_controller->set_target(0.0);
  }

}

/** Interpolate lookahead point for steering angle calculation */
bool MotionControlNode::compute_lookahead_point() {

  if (this->trajectory.points.empty()) {
    return false;
  }

  size_t next_waypoint_idx = find_next_waypoint(this->steering_controller->get_lookahead_distance(), this->current_position);
  TrajectoryPoint end = this->trajectory.points[next_waypoint_idx]; // point just outside of the lookahead distance
  TrajectoryPoint start = this->trajectory.points[next_waypoint_idx - 1];

  float lookahead_distance = steering_controller->get_lookahead_distance();
  TrajectoryPoint current_position = this->current_position;

  // TODO perform check that curve exists
  // TODO special logic if chosen waypoint is first or last in trajectory

  // Project vehicle's current position onto line between start and end points
  const tf2::Vector3 p_A(start.x, start.y, 0.0);
  const tf2::Vector3 p_B(end.x, end.y, 0.0);
  const tf2::Vector3 p_C(current_position.x, current_position.y, 0.0);
  const tf2::Vector3 AB = p_B - p_A;
  const tf2::Vector3 AC = p_C - p_A;
  const tf2::Vector3 p_D = p_A + AC.dot(AB) / AB.dot(AB) * AB;
  const double dist_CD = (p_D - p_C).length();

  // set lookahead point based on intersection
  //std::cout << "DIST: " << dist_CD << std::endl;

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
    
    if (this->trajectory.points.empty()) {
      // Error occurred, return old angle
      return this->steering_controller->get_steering_angle();
    }
    
    this->steering_controller->compute_curvature();
    return this->steering_controller->compute_steering_angle();
}

void MotionControlNode::visualize_markers(std::string frame_id, rclcpp::Time time) {

  if (this->trajectory.points.empty()) {
      return;
  }

  MarkerArray recorded_markers {};

  Marker trajectory_mark;
  trajectory_mark.type = Marker::LINE_STRIP;
  trajectory_mark.header.stamp = time;
  trajectory_mark.header.frame_id = frame_id;
  trajectory_mark.ns = "trajectory";
  trajectory_mark.action = Marker::ADD;

  trajectory_mark.scale.x = 0.3;
  trajectory_mark.frame_locked = true;

  trajectory_mark.color.r = 0.2588;
  trajectory_mark.color.g = 0.8784;
  trajectory_mark.color.b = 0.7725;
  trajectory_mark.color.a = 1.0;

  for(size_t i = 0; i < this->trajectory.points.size(); i++) {

    std::shared_ptr<TrajectoryPoint> point_ptr {new TrajectoryPoint};
    *point_ptr = this->trajectory.points[i];

    geometry_msgs::msg::Point p;
    p.x = point_ptr->x;
    p.y = point_ptr->y;
    p.z = point_ptr->z;

    trajectory_mark.points.push_back(p);
  }

  Marker circle_mark;
  circle_mark.type = Marker::CYLINDER;
  circle_mark.header.stamp = time;
  circle_mark.header.frame_id = frame_id;
  circle_mark.ns = "lookahead_circle";
  circle_mark.action = Marker::ADD;

  circle_mark.scale.x = this->steering_controller->get_lookahead_distance() * 2; // Diameter
  circle_mark.scale.y = this->steering_controller->get_lookahead_distance() * 2; // Diameter width of ellipse
  circle_mark.scale.z = 0.1;
  circle_mark.frame_locked = true;

  circle_mark.pose.position.x = this->current_position.x;
  circle_mark.pose.position.y = this->current_position.y;
  circle_mark.pose.position.z = this->current_position.z;

  circle_mark.color.r = 0.96;
  circle_mark.color.g = 0.831;
  circle_mark.color.b = 0.153;
  circle_mark.color.a = 0.7;


  Marker lookahead_point_mark;
  lookahead_point_mark.type = Marker::POINTS;
  lookahead_point_mark.header.stamp = time;
  lookahead_point_mark.header.frame_id = frame_id;
  lookahead_point_mark.ns = "lookahead_point";
  lookahead_point_mark.action = Marker::ADD;

  lookahead_point_mark.scale.x = 1.0;
  lookahead_point_mark.scale.y = 1.0;
  lookahead_point_mark.scale.z = 1.0;
  lookahead_point_mark.frame_locked = true;

  lookahead_point_mark.color.r = 1.0;
  lookahead_point_mark.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = this->steering_controller->get_lookahead_point_x();
  p.y = this->steering_controller->get_lookahead_point_y();
  lookahead_point_mark.points.push_back(p);

  //TODO: draw the steering arc between the vehicle and lookahead point using r = 1 / curvature

  recorded_markers.markers.push_back(circle_mark);
  recorded_markers.markers.push_back(trajectory_mark);
  recorded_markers.markers.push_back(lookahead_point_mark);
  this->marker_array_publisher->publish(recorded_markers);
  
  std::vector<Marker>().swap(recorded_markers.markers); // De-allocate markers
}

float MotionControlNode::distance(TrajectoryPoint p1, TrajectoryPoint p2) {
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

int MotionControlNode::steering_direction() {

  if (this->trajectory.points.empty()) {
    return 0;
  }

  size_t closest_point_idx = find_closest_point();
  TrajectoryPoint start = this->trajectory.points[closest_point_idx];
  TrajectoryPoint end = this->trajectory.points[closest_point_idx + 1];


  TrajectoryPoint P = this->current_position;

  float d = (P.x - start.x) * (end.y - start.y) - (P.y - start.y) * (end.x - start.x);

  if (d > 0) {
    std::cout << "RIGHT OF LINE" << std::endl;
    return -1;
  } else if (d < 0) {
    std::cout << "LEFT OF LINE" << std::endl;
    return 1;
  }


}