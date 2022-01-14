/*
 * Package:   nova_gps
 * Filename:  ConcreteGPSInterface.cpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "nova_gps/ConcreteGPSInterface.hpp"
#include "nova_gps/types.hpp"
#include "string.h"
#define GPS_I2C_ADDRESS 0x42

#define BYTES_LO 0xFD
#define BYTES_HI 0xFE
#define DATA_REGISTER 0xFF

using namespace Nova::GPS;

ConcreteGPSInterface::ConcreteGPSInterface() {
  this->i2c_interface = nullptr;
}

ConcreteGPSInterface::ConcreteGPSInterface(const std::string & interface_name) {
  this->open(interface_name);
}

ConcreteGPSInterface::~ConcreteGPSInterface() {
  this->close();
}

void ConcreteGPSInterface::close() {
  if(this->i2c_interface) {
    this->i2c_interface->close();
  }
}

void ConcreteGPSInterface::open(const std::string & interface_name) {
  // if we already have an interface, close it
  this->close();
  this->i2c_interface = std::make_unique<Nova::I2C::I2CInterface>(Nova::I2C::I2CInterface(GPS_I2C_ADDRESS, interface_name));
}

bool ConcreteGPSInterface::has_message() {
  return this->nmea_message_buffer.size() > 0;
}

std::unique_ptr<Nova::UBX::UBXMessage> ConcreteGPSInterface::get_message() {
  auto ptr = std::move(this->nmea_message_buffer.back());
  this->nmea_message_buffer.pop_back();
  return ptr;
}



bool ConcreteGPSInterface::gather_messages() {
  if(!this->i2c_interface) {
    throw std::runtime_error("No I2C interface opened before read");
  }

  uint16_t message_buffer_size = this->i2c_interface->read_word_pair(BYTES_LO, BYTES_HI);

  // if we got back FFFF, there's a problem. todo signal
  if(message_buffer_size == 0xFFFF || message_buffer_size == 0x0000) {
    // please try again later
    return false;
  }

  // in theory we should be checking for 0xFFs throughout this process, but it's unlikely.
  auto raw_block = this->i2c_interface->read_block(DATA_REGISTER, message_buffer_size);
  auto nmea_messages = Nova::UBX::parse_ubx_messages(*raw_block);
  for(auto message_iter = nmea_messages->begin(); message_iter < nmea_messages->end(); message_iter++) {
      this->nmea_message_buffer.push_back(std::move(*message_iter));
  }
  return true;
}

void ConcreteGPSInterface::write_config(Nova::ByteBuffer & config) {
  this->i2c_interface->write_block(config);
}
