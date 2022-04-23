/*
 * Package:   serial
 * Filename:  include/serial/serial_params.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <string>

namespace navigator {
namespace serial {

struct serial_params {
  std::string device_name;
};

}
}
