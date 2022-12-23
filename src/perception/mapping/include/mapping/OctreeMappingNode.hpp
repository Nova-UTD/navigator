/*
 * Package:   mapping
 * Filename:  OctreeMappingNode.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>

namespace navigator
{
  namespace perception
  {

    class OctreeMappingNode : public rclcpp::Node
    {
    public:
      OctreeMappingNode();
      virtual ~OctreeMappingNode();

    private:
      // std::unique_ptr<navigator::can_interface::CanBus> can_bus;
      // rclcpp::TimerBase::SharedPtr incoming_message_timer;
      // rclcpp::Publisher<voltron_msgs::msg::CanFrame>::SharedPtr incoming_message_publisher;
      // rclcpp::Subscription<voltron_msgs::msg::CanFrame>::SharedPtr outgoing_message_subscription;
    };

  }
}
