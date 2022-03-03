/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuit.cpp
 * Authors:   Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include <cmath>
#include <memory>
#include <iostream>
#include <tf2/utils.h>



#include "nova_pure_pursuit/PurePursuit.hpp"

using namespace Nova::PurePursuit;

using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;


PurePursuit::PurePursuit(float lookahead_distance) {
    
    closest_point = TrajectoryPoint();
    lookahead_point = TrajectoryPoint();
    
    this->lookahead_distance = lookahead_distance;
    this->curvature = 0.0;
    this->steering_angle = 0.0;

}

PurePursuit::~PurePursuit() {}


double PurePursuit::get_steering_angle(Trajectory cur_trajectory) {
    
    if(cur_trajectory.points.size() == 0) {
        return this->steering_angle; // error occurred, return old angle
    }

    this->trajectory = cur_trajectory;

    // Find first point outside lookahead distance to use to find lookahead point
    int next_waypoint_idx;

    for(size_t i = 0; i < cur_trajectory.points.size(); i++) {
      
      auto waypoint = cur_trajectory.points[i];
      auto currentPosition = TrajectoryPoint(); // fill-in get current location logic
      double distance = sqrt(pow(currentPosition.x - waypoint.x, 2) + pow(currentPosition.y - waypoint.y, 2));
      
      // first waypoint to be greater than lookahead distance
      if (distance > lookahead_distance) {
        next_waypoint_idx = i;
      }

    }

    // TODO perform check that curve exists
    // TODO special logic if chosen waypoint is first or last in trajectory
    
    // call function to set lookahead point
    set_lookahead_point(next_waypoint_idx);
    
    compute_curvature();
    compute_steering_angle();
    return this->steering_angle;
}

void PurePursuit::compute_curvature() {
    double denominator = pow(lookahead_point.x, 2) + pow(lookahead_point.y, 2);
    double numerator = 2.0 * lookahead_point.x;
    
    if (denominator != 0) {
        this->curvature = numerator / denominator;
    } else {
        this->curvature = numerator > 0 ? MIN_CURVATURE : -MIN_CURVATURE;
    }
}

void PurePursuit::compute_steering_angle() {
    this->steering_angle = atan(WHEEL_BASE * this->curvature);
}

/** Adaptative lateral control -> might use later */
void PurePursuit::set_lookahead_distance(float lookahead_distance) {
    this->lookahead_distance = lookahead_distance;
}

/** Steering effort calc -> might use later */
double PurePursuit::compute_steering_effort() { 
    return 0.0;
}

/** Interpolate lookahead point for steering angle calculation */
bool PurePursuit::set_lookahead_point(int next_waypoint_idx) {

  const TrajectoryPoint start = trajectory.points[next_waypoint_idx - 1];
  const TrajectoryPoint end = trajectory.points[next_waypoint_idx];
  auto currentPosition = TrajectoryPoint(); // fill-in get current location logic

  // Project vehicle's current position onto line between start and end points
  const tf2::Vector3 p_A(start.x, start.y, 0.0);
  const tf2::Vector3 p_B(end.x, end.y, 0.0);
  const tf2::Vector3 p_C(currentPosition.x, currentPosition.y, 0.0);
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
    lookahead_point.x = p_D.getX();
    lookahead_point.y = p_D.getY();
  } else {

    // two intersections, take intersection in front of vehicle
    double s = sqrt(pow(lookahead_distance, 2) - pow(dist_CD, 2));
    tf2::Vector3 p_E = p_D + s * AB.normalized();
    tf2::Vector3 p_F = p_D - s * AB.normalized();

    if ((p_B - p_E).length2() < AB.length2()) {
      lookahead_point.x = p_E.getX();
      lookahead_point.y = p_E.getY();
    } else if ((p_B - p_F).length2() < AB.length2()) {
      lookahead_point.x = p_F.getX();
      lookahead_point.y = p_F.getY();
    }

  }

  return true;
}