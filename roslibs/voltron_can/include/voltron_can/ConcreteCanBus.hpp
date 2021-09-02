/*
 * Package:   voltron_can
 * Filename:  ConcreteCanBus.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// A real, funcitoning CAN bus in an object-oriented package

#pragma once

#include <memory>
#include <string>

#include "CanBus.hpp"
#include "CanFrame.hpp" // Our object-oriented CAN frame representation

namespace Voltron {
  namespace Can {
    class ConcreteCanBus : public CanBus {
    public:
      ConcreteCanBus(); // Initialize without opening
      ConcreteCanBus(const std::string & interface_name); // Initialize and open
      virtual ~ConcreteCanBus();

      // Methods required to implement CanBus
      virtual bool is_frame_ready() override; // Whether a frame is ready
      virtual std::unique_ptr<CanFrame> read_frame() override; // Read a frame from the bus
      virtual void write_frame(const CanFrame & frame) override; // Write a frame to the bus
      virtual void open(const std::string & interface_name) override; // Open an interface
      virtual void close() override; // Close the open interface
      virtual bool is_open() override; // Whether or not the bus is open
    private:
      int raw_socket;
      std::string interface_name;
    };
  }
}
