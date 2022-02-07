/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuit.cpp
 * Authors:   Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include <chrono>
#include <memory>
#include <utility>
#include <cmath>
#include <string>

#include "nova_pure_pursuit/PurePursuit.hpp"

using namespace Nova::PurePursuit;


PurePursuit::PurePursuit(float lookahead_distance) {
    
    this->trajectory = voltron_msgs::msg::Trajectory();
    this->closest_point = voltron_msgs::msg::TrajectoryPoint();
    this->lookahead_point = voltron_msgs::msg::TrajectoryPoint();
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