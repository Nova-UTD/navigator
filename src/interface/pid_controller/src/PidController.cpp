/*
 * Package:   pid_controller
 * Filename:  PidController.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <chrono>
#include <memory>
#include <utility>

#include "pid_controller/PidController.hpp"

using namespace Voltron::PidController;

PidController::PidController(float KP, float KI, float KD, float time_delta_cap_seconds) {
  this->KP = KP;
  this->KI = KI;
  this->KD = KD;
  this->time_delta_cap_seconds = time_delta_cap_seconds;
  this->target = 0;
  this->measurement = 0;
  this->integral = 0;
  this->last_error = 0;
}

PidController::~PidController() {}

void PidController::set_target(float target) {this->target = target;}
void PidController::set_measurement(float measurement) {this->measurement = measurement;}

float PidController::compute(float time_delta_seconds) {
  if(time_delta_seconds > this->time_delta_cap_seconds) {
    time_delta_seconds = time_delta_cap_seconds;
  }

  float error = this->target - measurement;

  float P = KP * error;

  this->integral += (error * time_delta_seconds);
  float I = KI * this->integral;

  float derivative = (this->last_error - error) / time_delta_seconds;
  this->last_error = error;
  float D = KD * derivative;

  return P + I + D;
}
