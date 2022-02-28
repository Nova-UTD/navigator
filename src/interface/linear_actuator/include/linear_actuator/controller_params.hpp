/*
 * Package:   linear_actuator
 * Filename:  include/linear_actuator/controller_params.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "linear_actuator/types.hpp"

namespace navigator {
namespace linear_actuator {

struct controller_params {
  int disengaged_position;
  int engaged_position;
  can_id_t command_id;
};

}
}
