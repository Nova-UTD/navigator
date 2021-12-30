/*
 * Package:   voltron_test_utils
 * Filename:  TestPublisher.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp" // For the ROS node
#include <string>
#include <unistd.h> // usleep

namespace Voltron {
namespace TestUtils {

template <typename MsgType> class TestPublisher {
public:
  TestPublisher(std::string topic) {
    this->node = std::make_shared<TestPublisherNode>(topic);
  }

  virtual ~TestPublisher() {}

  void send_message(MsgType message) {
    this->node->send_message(message);
    rclcpp::spin_some(this->node);
    usleep(1000);
  }

private:
  class TestPublisherNode : public rclcpp::Node {
  public:
    TestPublisherNode(std::string topic) : Node("test_publisher_node_" + topic) {
      this->publisher = this->create_publisher<MsgType>(topic, 64);
    }

    virtual ~TestPublisherNode() {}

    void send_message(MsgType message) {
      this->publisher->publish(message);
    }

  private:
    typename rclcpp::Publisher<MsgType>::SharedPtr publisher;
  };

  std::shared_ptr<TestPublisherNode> node;
};

}
}
