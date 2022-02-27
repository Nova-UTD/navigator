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

#include "nova_pure_pursuit/PurePursuit.hpp"

using namespace Nova::PurePursuit;


PurePursuit::PurePursuit(float lookahead_distance) {
    
    this->closest_point_x = 0.0;
    this->closest_point_y = 0.0;
    
    this->lookahead_point_x = 0.0;
    this->lookahead_point_y = 0.0;

    this->lookahead_distance = lookahead_distance;
    this->curvature = 0.0;
    this->steering_angle = 0.0;

}

PurePursuit::~PurePursuit() {}


double PurePursuit::get_steering_angle(voltron_msgs::msg::Trajectory cur_trajectory) {
    
    if(cur_trajectory.points.size() == 0) {
        return this->steering_angle; // error occurred, return old angle
    }

    this->trajectory = cur_trajectory;

    // do we need to localize?
    calcRelativeCoordinate();
    this->closest_point = this->trajectory.points[0];
    
    set_lookahead_point();
    compute_curvature();
    compute_steering_angle();
    return this->steering_angle;
}

void PurePursuit::set_lookahead_point() {
    lookahead_point = trajectory.points[1];
}

void PurePursuit::compute_curvature() {
    double denominator = pow(lookahead_point_x, 2) + pow(lookahead_point_y, 2);
    double numerator = 2.0 * lookahead_point_x;
    
    if (denominator != 0) {
        this->curvature = numerator / denominator;
    } else {
        this->curvature = numerator > 0 ? MIN_CURVATURE : -MIN_CURVATURE;
    }
}

void PurePursuit::compute_steering_angle() {
    this->steering_angle = atan(WHEEL_BASE * this->curvature);
}

double PurePursuit::get_steering_angle() {
    
    // if(cur_trajectory.points.size() == 0) {
    //     return this->steering_angle; // error occurred, return old angle
    // }

    // this->trajectory = cur_trajectory;
    // this->closest_point = this->trajectory.points[0];
    set_lookahead_point(0.0, 0.0);
    compute_curvature();
    compute_steering_angle();
    return this->steering_angle;
}

/** Adaptative lateral control -> might use later */
void PurePursuit::set_lookahead_distance(float lookahead_distance) {
    this->lookahead_distance = lookahead_distance;
}

/** Steering effort calc -> might use later */
double PurePursuit::compute_steering_effort() { 
    return 0.0;
}







bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) const
{
    
  const double search_radius = lookahead_distance_;
  const geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
  const geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // project ego vehicle's current position at C onto the line at D in between two waypoints A and B.
  const tf::Vector3 p_A(start.x, start.y, 0.0);
  const tf::Vector3 p_B(end.x, end.y, 0.0);
  const tf::Vector3 p_C(current_pose_.position.x, current_pose_.position.y, 0.0);
  const tf::Vector3 AB = p_B - p_A;
  const tf::Vector3 AC = p_C - p_A;
  const tf::Vector3 p_D = p_A + AC.dot(AB) / AB.dot(AB) * AB;
  const double dist_CD = (p_D - p_C).length();

  bool found = false;
  tf::Vector3 final_goal;
  // Draw a circle centered at p_C with a radius of search_radius
  if (dist_CD > search_radius)
  {
    // no intersection in between the circle and AB
    found = false;
  }
  else if (dist_CD == search_radius)
  {
    // one intersection
    final_goal = p_D;
    found = true;
  }
  else
  {
    // two intersections
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(dist_CD, 2));
    tf::Vector3 p_E = p_D + s * AB.normalized();
    tf::Vector3 p_F = p_D - s * AB.normalized();

    // verify whether these two points lie on line segment AB
    if ((p_B - p_E).length2() < AB.length2())
    {
      final_goal = p_E;
      found = true;
    }
    else if ((p_B - p_F).length2() < AB.length2())
    {
      final_goal = p_F;
      found = true;
    }
    else
    {
      found = false;
    }
  }

  if (found)
  {
    next_target->x = final_goal.x();
    next_target->y = final_goal.y();
    next_target->z = current_pose_.position.z;
  }

  return found;
}