/*
 * Package:   voltron_can
 * Filename:  CanBus.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// Abstract class to represent a CAN bus in an object-oriented way

#pragma once

#include <memory>
#include <string>

#include "CanFrame.hpp" // Our object-oriented CAN frame representation

namespace Voltron {
  namespace Can {
    class CanBus {
    public:
      virtual ~CanBus() {} // Make it extendable
      virtual bool is_frame_ready() = 0; // Whether a frame is ready
      virtual std::unique_ptr<CanFrame> read_frame() = 0; // Read a frame from the bus
      virtual void write_frame(const CanFrame & frame) = 0; // Write a frame to the bus
      virtual void open(const std::string & interface_name) = 0; // Open an interface
      virtual void close() = 0; // Close the open interface
      virtual bool is_open() = 0; // Whether or not the bus is open
    };
  }
}
