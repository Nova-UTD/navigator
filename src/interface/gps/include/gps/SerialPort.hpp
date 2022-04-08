/*
 * Package:   gps
 * Filename:  include/gps/SerialPort.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <string>

namespace navigator {
namespace gps {

class SerialPort final {
public:
  SerialPort(const std::string & device_name);
  std::string get_line();

private:
  int descriptor;
};

}
}
