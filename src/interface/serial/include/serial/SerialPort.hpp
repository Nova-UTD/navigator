/*
 * Package:   serial
 * Filename:  include/serial/SerialPort.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <optional>
#include <string>

namespace navigator {
namespace serial {

class SerialPort final {
public:
  SerialPort(const std::string & device_name);
  ~SerialPort();
  std::optional<std::string> get_line();
  void send(std::string message);

private:
  int descriptor;
  std::string buffer;
};

}
}
