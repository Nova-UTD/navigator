/*
 * Package:   can_interface
 * Filename:  CanInterfaceNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nova_msgs/msg/can_frame.hpp"
#include "CanBus.hpp"

namespace navigator {
namespace can_interface {

class CanInterfaceNode : public rclcpp::Node {
public:
  CanInterfaceNode(const std::string & interface_name);
  virtual ~CanInterfaceNode();

private:
  void send_frame(const nova_msgs::msg::CanFrame::SharedPtr msg);
  void check_incoming_messages();
  void receive_frame();


  std::unique_ptr<navigator::can_interface::CanBus> can_bus;
  rclcpp::TimerBase::SharedPtr incoming_message_timer;
  rclcpp::Publisher<nova_msgs::msg::CanFrame>::SharedPtr incoming_message_publisher;
  rclcpp::Subscription<nova_msgs::msg::CanFrame>::SharedPtr outgoing_message_subscription;
};

}
}
