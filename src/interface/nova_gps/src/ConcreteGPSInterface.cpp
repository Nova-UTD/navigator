/*
 * Package:   nova_gps
 * Filename:  ConcreteGPSInterface.cpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "nova_gps/ConcreteGPSInterface.hpp"
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

bool ConcreteGPSInterface::has_nmea_message() {
  return this->nmea_message_buffer.size() > 0;
}

std::unique_ptr<NMEAMessage> ConcreteGPSInterface::get_nmea_message() {
  auto ptr = std::move(std::make_unique<NMEAMessage>(this->nmea_message_buffer.back()));
  this->nmea_message_buffer.pop_back();
  return ptr;
}

std::shared_ptr<std::vector<std::string>> split_ascii_buffer(const std::vector<char> & buffer) {
  std::vector<std::string> substrings;
  std::string buf_as_string(buffer.begin(), buffer.end());

  substrings.reserve(buffer.size() / 84); // NMEA0183 says messages are capped at 84 characters, but this is frequently violated.

  int substring_start = 0;
  int substring_end = buf_as_string.find('\n'); // messages end with a CR-LF
  while(substring_end != -1) {
    substrings.push_back(buf_as_string.substr(substring_start, substring_end - substring_start - 1)); // trim the CR as well
    substring_start = substring_end + 1;
    substring_end = buf_as_string.find('\n', substring_start);
  }
  
  return std::make_shared<std::vector<std::string>>(substrings);
}

std::shared_ptr<std::vector<NMEAMessage>> parse_nmea_messages(std::shared_ptr<std::vector<std::string>> raw_messages) {
    // todo: parse into a number of frames with data validation based on message type
    return raw_messages;
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
  // no UBX message support (yet)
  auto raw_block = this->i2c_interface->read_block(DATA_REGISTER, message_buffer_size);
  /*std::shared_ptr<std::vector<NMEAMessage>> nmea_messages = parse_nmea_messages(split_ascii_buffer(*raw_block));
  for(auto message_iter = nmea_messages->begin(); message_iter < nmea_messages->end(); message_iter++) {
      this->nmea_message_buffer.push_back(*message_iter);
  }*/
  return true;
}

void ConcreteGPSInterface::write_config(Nova::ByteBuffer & config) {
  this->i2c_interface->write_block(config);
}
