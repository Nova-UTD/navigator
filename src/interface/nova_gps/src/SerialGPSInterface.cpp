#include "nova_gps/SerialGPSInterface.hpp"
#include "nova_gps/types.hpp"
#include "string.h"
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace Nova::GPS;

int close_handle(int handle) {
  return close(handle);
}

int open_serial(const std::string & interface) {
  return open(interface.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
}

SerialGPSInterface::SerialGPSInterface() {
  this->serial_device_handle = 0;
}

SerialGPSInterface::SerialGPSInterface(const std::string & interface_name) {
  this->open(interface_name);
}

SerialGPSInterface::~SerialGPSInterface() {
  this->close();
}

void SerialGPSInterface::close() {
  if(this->serial_device_handle) {
    close_handle(serial_device_handle);
    serial_device_handle = 0;
  }
}

void SerialGPSInterface::open(const std::string & interface_name) {
  // if we already have an interface, close it
  this->close();
  this->serial_device_handle = open_serial(interface_name);
  if(this->serial_device_handle < 0) {
      perror("Couldn't open serial port");
      throw std::runtime_error("");
  }
  struct termios tty;
  tcgetattr(this->serial_device_handle, &tty);
  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  cfmakeraw(&tty);
  if(tcsetattr(this->serial_device_handle, TCSANOW, &tty) != 0) {
    perror("Couldn't configure serial port");
    throw std::runtime_error("");
  }
}

bool SerialGPSInterface::has_message() {
  return this->nmea_message_buffer.size() > 0;
}

std::unique_ptr<Nova::UBX::UBXMessage> SerialGPSInterface::get_message() {
  auto ptr = std::move(this->nmea_message_buffer.back());
  this->nmea_message_buffer.pop_back();
  return ptr;
}

void SerialGPSInterface::enqueue_outgoing_message(std::unique_ptr<Nova::UBX::UBXMessage> message) {
  this->send_queue.push_back(std::move(message));
}

bool SerialGPSInterface::send_messages() {
  /*if(!this->serial_device_handle) {
    throw std::runtime_error("No serial interface opened before write");
  }
  while(this->send_queue.size() > 0) {
    auto msg = std::move(this->send_queue.back());
    this->send_queue.pop_back();
    this->i2c_interface->write_byte(0xB5);
    this->i2c_interface->write_byte(0x62);
    this->i2c_interface->write_byte(msg->mclass);
    this->i2c_interface->write_byte(msg->id);
    this->i2c_interface->write_byte(msg->data->size() & 0xFF);
    this->i2c_interface->write_byte((msg->data->size() >> 8) & 0xFF);
    this->i2c_interface->write_block(*msg->data);
    this->i2c_interface->write_byte(msg->checksum & 0xFF);
    this->i2c_interface->write_byte((msg->checksum >> 8) & 0xFF);
  }*/
  throw std::runtime_error("not implemented");
  return true;
}

bool SerialGPSInterface::gather_messages() {
  if(!this->serial_device_handle) {
    throw std::runtime_error("No serial interface opened before read");
  }

  // in theory we should be checking for 0xFFs throughout this process, but it's unlikely.
  std::unique_ptr<Nova::ByteBuffer> buf = std::make_unique<Nova::ByteBuffer>(255);
  int bytes = read(this->serial_device_handle, &buf->operator[](0), buf->size());
  auto nmea_messages = Nova::UBX::parse_ubx_messages(*buf);
  for(auto message_iter = nmea_messages->begin(); message_iter < nmea_messages->end(); message_iter++) {
      this->nmea_message_buffer.push_back(std::move(*message_iter));
  }
  return true;
}