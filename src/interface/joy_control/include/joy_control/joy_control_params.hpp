/*
 * Package:   joy_control
 * Filename:  include/joy_control/joy_control_params.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

namespace navigator {
namespace joy_control {

struct joy_control_params {
  int throttle_axis;
  int brake_axis;
  int steering_axis;
  int enable_button;
  float max_steering_angle;
};

}
}
