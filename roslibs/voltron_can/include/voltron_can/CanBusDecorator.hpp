/*
 * Package:   voltron_can
 * Filename:  CanBusDecorator.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// Base class for all CAN bus decorators

#pragma once

#include <memory>
#include <string>

#include "CanFrame.hpp"
#include "CanBus.hpp"

namespace Voltron {
  namespace Can {
    class CanBusDecorator : public CanBus {
    public:
      CanBusDecorator(std::unique_ptr<CanBus> && delegate);
      virtual ~CanBusDecorator() {}
      virtual bool is_frame_ready() override; // Whether a frame is ready
      virtual std::unique_ptr<CanFrame> read_frame() override; // Read a frame from the bus
      virtual void write_frame(const CanFrame & frame) override; // Write a frame to the bus
      virtual void open(const std::string & interface_name) override; // Open an interface
      virtual void close() override; // Close the open interface
      virtual bool is_open() override; // Whether or not the bus is open

    protected:
      std::unique_ptr<CanBus> delegate;
    };
  }
}
