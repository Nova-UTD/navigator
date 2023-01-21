/*
 * Package:   can_interface
 * Filename:  CanFrame.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// A simple class to represent a CAN-frame, independent of many of the
// details found in the can_frame struct provided by the system.

#pragma once

#include <cstdint> // Fixed-width integers
#include <linux/can.h> // struct can_frame
#include <memory> // std::unique_ptr

namespace navigator {
  namespace can_interface {
    class CanFrame final {
    public:
      typedef uint32_t identifier_t;
      typedef uint64_t data_t;

      // Construct from raw data
      CanFrame(identifier_t identifier, data_t data);

      // Construct from a system-provided can frame
      CanFrame(const struct can_frame & frame_struct);

      ~CanFrame();

      // Members can be freely read
      identifier_t get_identifier() const;
      data_t get_data() const;

      // Convert this to the struct provided by the system
      std::unique_ptr<can_frame> to_system_frame() const;

    private:
      identifier_t identifier;
      data_t data;
    };
  }
}
