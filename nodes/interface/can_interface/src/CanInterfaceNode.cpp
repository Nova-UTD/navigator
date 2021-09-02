/*
 * Package:   can_interface
 * Filename:  CanBusInterface.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <chrono> // Time literals
#include <functional> // Callbacks
#include <iostream> // I/O in main()
#include <string> // Because we are not barbarians

#include "rclcpp/rclcpp.hpp" // ROS node

#include "voltron_msgs/msg/can_frame.hpp" // CAN frame messages
#include "can_interface/CanBus.hpp" // CAN interface
#include "can_interface/ConcreteCanBus.hpp"

#include "can_interface/CanInterfaceNode.hpp" // Header for this class

using namespace std::chrono_literals;
using Voltron::Can::CanInterfaceNode;
using std::placeholders::_1;

const auto receive_frequency = 15ms;

CanInterfaceNode::CanInterfaceNode(const std::string & interface_name)
  : Node("can_interface_node_" + interface_name) {

  this->can_bus = std::make_unique<Voltron::Can::ConcreteCanBus>(interface_name);

  // Set up the timer
  this->incoming_message_timer = this->create_wall_timer
    (receive_frequency, bind(& CanInterfaceNode::check_incoming_messages, this));

  // Set up the publisher. Buffer up to 64 since the CAN bus could get fairly busy
  this->incoming_message_publisher = this->create_publisher<voltron_msgs::msg::CanFrame>
    ("incoming_can_frames_" + interface_name, 64);

  // Subscribe to outgoing CAN messages
  this->outgoing_message_subscription =
    this->create_subscription<voltron_msgs::msg::CanFrame>
    ("outgoing_can_frames_" + interface_name, 64,
     bind(& CanInterfaceNode::send_frame, this, _1));
}

CanInterfaceNode::~CanInterfaceNode() {
    
}

void CanInterfaceNode::send_frame(const voltron_msgs::msg::CanFrame::SharedPtr msg) {
  this->can_bus->write_frame(Voltron::Can::CanFrame(msg->identifier, msg->data));
}

void CanInterfaceNode::check_incoming_messages() {
  while(this->can_bus->is_frame_ready()) {
    this->receive_frame();
  }
}

void CanInterfaceNode::receive_frame() {
  std::unique_ptr<Voltron::Can::CanFrame> frame = this->can_bus->read_frame();
  voltron_msgs::msg::CanFrame message;
  message.identifier = frame->get_identifier();
  message.data = frame->get_data();
  this->incoming_message_publisher->publish(message);
}
