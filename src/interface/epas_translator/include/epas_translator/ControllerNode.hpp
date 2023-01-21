/*
 * Package:   epas_translator
 * Filename:  ControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <chrono> // Time literals
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nova_msgs/msg/can_frame.hpp" // CAN messages
#include "std_msgs/msg/float32.hpp" // UInt8 messages

typedef uint32_t can_id_t;
typedef uint64_t can_data_t;

using namespace std::chrono_literals;

namespace Voltron {
namespace EpasSteering {

// How often to publish the steering message
// The "ideal" range from the EPAS docs is 50-200Hz
constexpr auto control_message_frequency = 20ms;

// Which steering map to use (1-5)
// Higher values = faster
// I don't know if there's any reason not to just use 5?
constexpr uint8_t steering_map = 5;

// The CAN identifier to use when publishing control messages
constexpr can_id_t epas_control_can_identifier = 0x296;

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  virtual ~ControllerNode();

private:
  void send_control_message();
  void update_power(const std_msgs::msg::Float32::SharedPtr message);

  uint8_t power = 255/2;
  rclcpp::Publisher<nova_msgs::msg::CanFrame>::SharedPtr can_publisher;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr power_subscription;
  rclcpp::TimerBase::SharedPtr control_timer;
};
}
}
