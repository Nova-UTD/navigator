/*
 * Package:   serial
 * Filename:  include/serial/SerialNode
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial_params.hpp"
#include "serial/SerialPort.hpp"

namespace navigator {
namespace serial {

class SerialNode final : public rclcpp::Node {
public:
  SerialNode();
  SerialNode(serial_params params);

private:
  void check_messages();
  void send_message(std_msgs::msg::String::SharedPtr message);
  void init();

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  std::unique_ptr<SerialPort> serial_port;
  serial_params params;
};

}
}
