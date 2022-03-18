/*
 * Package:   nova_motion_control
 * Filename:  PurePursuit.cpp
 * Authors:   Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include <cmath>
#include <memory>
#include <iostream>


#include "nova_motion_control/PurePursuit.hpp"

using namespace Nova::PurePursuit;


PurePursuit::PurePursuit(float lookahead_distance) {
    
    this->closest_point_x = 0.0;
    this->closest_point_y = 0.0;

    this->lookahead_point_x = 0.0;
    this->lookahead_point_y = 0.0;
    
    this->lookahead_distance = lookahead_distance;
    this->curvature = 0.0;
    this->steering_angle = 0.0;

    this->displacement_error = 0.0;

}

PurePursuit::~PurePursuit() {}

double PurePursuit::get_steering_angle() { return steering_angle; }
float PurePursuit::get_lookahead_distance() { return lookahead_distance; }

float PurePursuit::get_lookahead_point_x() { return lookahead_point_x; }
float PurePursuit::get_lookahead_point_y() { return lookahead_point_y; }

float PurePursuit::get_displacement_error() { return displacement_error; }

void PurePursuit::set_lookahead_point(float x, float y) {
    lookahead_point_x = x;
    lookahead_point_y = y;
}

void PurePursuit::set_displacement_error(float displacement) { this->displacement_error = displacement; }


float PurePursuit::compute_curvature() {
    double denominator = pow(lookahead_point_x, 2) + pow(lookahead_point_y, 2);
    double numerator = 2.0 * lookahead_point_x;
    
    if (denominator != 0) {
        this->curvature = numerator / denominator;
    } else {
        this->curvature = numerator > 0 ? MIN_CURVATURE : -MIN_CURVATURE;
    }

    //std::cout << "CURVATURE: " << this->curvature << std::endl;

    return curvature;
}

float PurePursuit::compute_steering_angle() {
    this->steering_angle = atan(WHEEL_BASE * this->curvature);
    return steering_angle;
}






/** Adaptative lateral control -> might use later */
void PurePursuit::set_lookahead_distance(float lookahead_distance) {
    this->lookahead_distance = lookahead_distance;
}

/** Steering effort calc -> might use later */
double PurePursuit::compute_steering_effort() { 
    return 0.0;
}