/*
 * Package:   pid_controller
 * Filename:  PidController.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// TODO: Make this an interface and extend with a concrete PID
// controller

#pragma once

#include <chrono>
#include <memory>

namespace Voltron {
namespace PidController {

class PidController {
public:
  PidController(float KP, float KI, float KD, float time_delta_cap_seconds);
  virtual ~PidController();

  virtual float compute(float time_delta_seconds);
  virtual void set_target(float target_angle);
  virtual void set_measurement(float measurement);

private:
  float KP;
  float KI;
  float KD;
  float time_delta_cap_seconds;

  float target;
  float measurement;

  float integral;
  float last_error;
};
}
}
