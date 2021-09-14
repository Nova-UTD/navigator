/*
 * Package:   can_interface
 * Filename:  CanInterfaceNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "voltron_msgs/msg/can_frame.hpp"
#include "CanBus.hpp"

namespace Voltron {
namespace Can {

class CanInterfaceNode : public rclcpp::Node {
public:
  CanInterfaceNode();
  CanInterfaceNode(const std::string & interface_name);
  virtual ~CanInterfaceNode();

private:
  void send_frame(const voltron_msgs::msg::CanFrame::SharedPtr msg);
  void check_incoming_messages();
  void receive_frame();
  void initialize(const std::string & interface_name);

  std::unique_ptr<Voltron::Can::CanBus> can_bus;
  rclcpp::TimerBase::SharedPtr incoming_message_timer;
  rclcpp::Publisher<voltron_msgs::msg::CanFrame>::SharedPtr incoming_message_publisher;
  rclcpp::Subscription<voltron_msgs::msg::CanFrame>::SharedPtr outgoing_message_subscription;
};

}
}
