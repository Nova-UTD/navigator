/*
 * Package:   can_interface
 * Filename:  CanBusInterface.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <chrono> // Time literals
#include <functional> // Callbacks
#include <iostream> // I/O in main()
#include <string> // Because we are not barbarians

#include "rclcpp/rclcpp.hpp" // ROS node

#include "nova_msgs/msg/can_frame.hpp" // CAN frame messages
#include "can_interface/CanBus.hpp" // CAN interface

#include "can_interface/CanInterfaceNode.hpp" // Header for this class

using namespace std::chrono_literals;
using navigator::can_interface::CanInterfaceNode;
using std::placeholders::_1;

const auto receive_frequency = 15ms;

CanInterfaceNode::CanInterfaceNode(const std::string & interface_name)
  : Node("can_interface") {

  this->can_bus = std::make_unique<navigator::can_interface::CanBus>(interface_name);

  // Set up the timer
  this->incoming_message_timer = this->create_wall_timer
    (receive_frequency, bind(& CanInterfaceNode::check_incoming_messages, this));

  // Set up the publisher. Buffer up to 64 since the CAN bus could get fairly busy
  this->incoming_message_publisher = this->create_publisher<nova_msgs::msg::CanFrame>
    ("can_interface_incoming_can_frames", 64);

  // Subscribe to outgoing CAN messages
  this->outgoing_message_subscription =
    this->create_subscription<nova_msgs::msg::CanFrame>
    ("can_interface_outgoing_can_frames", 64,
     bind(& CanInterfaceNode::send_frame, this, _1));
}

CanInterfaceNode::~CanInterfaceNode() {

}

void CanInterfaceNode::send_frame(const nova_msgs::msg::CanFrame::SharedPtr msg) {
  this->can_bus->write_frame(navigator::can_interface::CanFrame(msg->identifier, msg->data));
}

void CanInterfaceNode::check_incoming_messages() {
  while(this->can_bus->is_frame_ready()) {
    this->receive_frame();
  }
}

void CanInterfaceNode::receive_frame() {
  std::unique_ptr<navigator::can_interface::CanFrame> frame = this->can_bus->read_frame();
  nova_msgs::msg::CanFrame message;
  message.identifier = frame->get_identifier();
  message.data = frame->get_data();
  this->incoming_message_publisher->publish(message);
}
