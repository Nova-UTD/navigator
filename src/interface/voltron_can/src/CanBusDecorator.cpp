/*
 * Package:   voltron_can
 * Filename:  CanBusDecorator.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <memory>
#include <string>

#include "voltron_can/CanBusDecorator.hpp"
#include "voltron_can/CanBus.hpp"
#include "voltron_can/CanFrame.hpp"

using namespace Voltron::Can;

CanBusDecorator::CanBusDecorator(std::unique_ptr<CanBus> && delegate) {
  this->delegate = std::move(delegate);
}

bool CanBusDecorator::is_frame_ready() {
  return this->delegate->is_frame_ready();
}

std::unique_ptr<CanFrame> CanBusDecorator::read_frame() {
  return this->delegate->read_frame();
}

void CanBusDecorator::write_frame(const CanFrame & frame) {
  this->delegate->write_frame(frame);
}

void CanBusDecorator::open(const std::string & interface_name) {
  this->delegate->open(interface_name);
}

void CanBusDecorator::close() {
  this->delegate->close();
}

bool CanBusDecorator::is_open() {
  return this->delegate->is_open();
}
