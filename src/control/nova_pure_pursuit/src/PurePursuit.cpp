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

    this->closest_x = 0.0;
    this->closest_y = 0.0;

    this->lookahead_distance = lookahead_distance;
    this->lookahead_x = 0.0;
    this->lookahead_y = 0.0;

    this->curvature = 0.0;
    this->steering_angle = 0.0;
}

PurePursuit::~PurePursuit() {}


/**************Adaptive Lateral Control************
void PurePursuit::set_lookahead_distance(float lookahead_distance) {this->lookahead_distance = lookahead_distance;}
*/


double PurePursuit::get_steering_angle(voltron_msgs::msg::Trajectory trajectory) {
    return 0;
}

void PurePursuit::compute_curvature() {
    double denominator = pow(lookahead_x, 2) + pow(lookahead_y, 2);
    double numerator = 2.0 * lookahead_x;
    
    if (denominator != 0) {
        this->curvature = numerator / denominator;
    } else {
        this->curvature = numerator > 0 ? MIN_CURVATURE : -MIN_CURVATURE;
    }
}

void PurePursuit::compute_steering_angle() {
    double L = 0.1; // wheel-base
    compute_curvature();
    this->steering_angle = atan(L * this->curvature);
}

/** Might use in later versions */
// double PurePursuit::compute_steering_effort() { return compute_steering_angle();}

void PurePursuit::set_lookahead_point(float x, float y) {
    this->lookahead_x = x;
    this->lookahead_y = y;
}

void PurePursuit::set_closest_point(float x, float y) {
    this->closest_x = x;
    this->closest_y = y;
}
