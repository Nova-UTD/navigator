/*
 * Package:   can_interface
 * Filename:  ConcreteCanBus.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <cstring> // strcpy()
#include <linux/can.h> // CAN communication
#include <net/if.h> // Also for CAN communication
#include <string> // Because we are not barbarians
#include <stdexcept> // Runtime errors
#include <sys/ioctl.h> // More OS-level CAN bus stuff
#include <sys/select.h> // Allows checking whether messages are ready
#include <sys/socket.h> // To open the socket we need
#include <unistd.h> // Socket I/O

#include <iostream> // Temporary, used for debugging

#include "can_interface/ConcreteCanBus.hpp" // Obviously, we need the class header
#include "can_interface/CanFrame.hpp" // Object-oriented

using namespace Voltron::Can;

// Alias the OS's close() function
inline void close_socket(int socket) {
  close(socket);
}

ConcreteCanBus::ConcreteCanBus() {
  this->raw_socket = -1;
}

ConcreteCanBus::ConcreteCanBus(const std::string & interface_name) {
  this->raw_socket = -1;
  this->open(interface_name);
}

// Try to connect to the given interface name
void ConcreteCanBus::open(const std::string & interface_name) {
  this->interface_name = interface_name;
  // Set up the socket
  this->raw_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(this->raw_socket < 0) {
    switch(errno) { // TODO add exceptions for more error types
    default:
      throw std::runtime_error("Unknown error while opening raw socket: errno is " +
			       std::to_string(errno));
    }
  }
    
  // Connect the socket to the interface
  ifreq interface;
  strcpy(interface.ifr_name, interface_name.c_str());
  int result = ioctl(this->raw_socket, SIOCGIFINDEX, &(interface));
  if(result != 0) {
    switch(errno) { // TODO add exceptions for more error types
    default:
      throw std::runtime_error("Unknown error while connecting socket to interface" +
			       interface_name + ": errno is " + std::to_string(errno));
    }
  }

  // Bind the socket
  sockaddr_can address;
  address.can_family = AF_CAN;
  address.can_ifindex = interface.ifr_ifindex;
  result = bind(this->raw_socket, (struct sockaddr *) (&(address)), sizeof(address));
  if(result < 0) {
    switch(errno) { // TODO add exceptions for more error types
    default:
      throw std::runtime_error("Error while binding socket to interface " +
			       interface_name + ": errno is " + std::to_string(errno));
    }
  }
}

void ConcreteCanBus::close() {
  if(this->is_open()) {
    close_socket(this->raw_socket);
    this->raw_socket = -1;
  }
}

bool ConcreteCanBus::is_open() {
  return this->raw_socket > 0;
}

// If the socket was open, close it
ConcreteCanBus::~ConcreteCanBus() {
  if(this->is_open()) {
    this->close();
  }
}

bool ConcreteCanBus::is_frame_ready() {
  // Put the socket into a set, which is required by select()
  fd_set socket_set;
  FD_ZERO(&socket_set);
  FD_SET(this->raw_socket, &socket_set);

  // Make sure a message is ready
  // See "man select" for more information
  // TODO: Better error handling
  struct timeval timeout { 0, 0 };
  int select_result = select(this->raw_socket + 1, &socket_set, NULL, NULL, &timeout);
  if(select_result < 0) throw std::runtime_error("Error from select()");
  return select_result != 0; // Zero indicates timeout / not ready
}

std::unique_ptr<CanFrame> ConcreteCanBus::read_frame() {
  // Read a frame
  struct can_frame frame;
  int n_bytes = read(this->raw_socket, &frame, sizeof(struct can_frame));

  // TODO: Evaluate what to do here - maybe throw a custom exception
  if (n_bytes < (int)sizeof(struct can_frame)) {
    throw std::runtime_error("Read a partial CAN frame on interface " + this->interface_name);
  }

  return std::make_unique<CanFrame>(frame);
}

void ConcreteCanBus::write_frame(const CanFrame & frame) {
  int n_bytes = write(this->raw_socket, &(*frame.to_system_frame()), sizeof(struct can_frame));
  if(n_bytes < 1) {
    switch(errno) {
    default:
      throw std::runtime_error("Failed to deliver CAN frame on interface " +
			       this->interface_name);
    }
  }
  return;
}
