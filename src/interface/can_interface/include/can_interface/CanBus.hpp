/*
 * Package:   can_interface
 * Filename:  CanBus.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// A real, funcitoning CAN bus in an object-oriented package

#pragma once

#include <memory>
#include <string>

#include "CanFrame.hpp" // Our object-oriented CAN frame representation

namespace navigator {
  namespace can_interface {
    class CanBus final {
    public:
      CanBus(); // Initialize without opening
      CanBus(const std::string & interface_name); // Initialize and open
      ~CanBus();

      // Methods required to implement CanBus
      bool is_frame_ready(); // Whether a frame is ready
      std::unique_ptr<CanFrame> read_frame(); // Read a frame from the bus
      void write_frame(const CanFrame & frame); // Write a frame to the bus
      void open(const std::string & interface_name); // Open an interface
      void close(); // Close the open interface
      bool is_open(); // Whether or not the bus is open
    private:
      int raw_socket;
      std::string interface_name;
    };
  }
}
