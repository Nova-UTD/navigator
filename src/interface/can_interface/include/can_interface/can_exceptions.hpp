/*
 * Package:   can_interface
 * Filename:  can_exceptions.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// Describes various exceptions that can be thrown by the SocketCAN
// wrapper API

#include <stdexcept> // Base exceptions

class DeviceNotFoundException : public std::exception {
public:
  DeviceNotFoundException(const std::string & interface) {
    this->interface = interface;
  }

  const char * what() const throw() {
    return (this->interface + " could not be found").c_str();
  }

private:
  std::string interface;
};
