/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuit.cpp
 * Author:    Cristian Cruz
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


void PurePursuit::set_lookahead_point(float x, float y) {
    this->lookahead_x = x;
    this->lookahead_y = y;
}

void PurePursuit::set_closest_point(float x, float y) {
    this->closest_x = x;
    this->closest_y = y;
}

// Output should be in radians
float PurePursuit::compute_steering_angle() {

    // https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-geometric-lateral-control-pure-pursuit-44N7x
    // Coursera Lesson 2: Geometric Lateral Control - Pure Pursuit
    // Geometry for the vehicle's origin being in the rear

    float alpha = 1.0; // Direction angle of the rear wheel
    float curvature = (2 * asin(alpha)) / lookahead_distance;

    float L = 0.1; // Length of wheelbase? This may be a constant for class at initialization
    float steering_angle = atan(L * curvature);

    return steering_angle;
}

std::string PurePursuit::hello_world() {
    return "CRIS ROCKS ";
}