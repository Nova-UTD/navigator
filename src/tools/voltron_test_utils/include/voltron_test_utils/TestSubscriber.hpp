/*
 * Package:   voltron_test_utils
 * Filename:  TestSubscriber.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp" // Because this is a ROS node
#include <queue>
#include <string>
#include <unistd.h> // usleep

namespace Voltron {
namespace TestUtils {

template <typename MsgType> class TestSubscriber {
public:
  TestSubscriber(std::string topic) {
    this->node = std::make_shared<TestSubscriberNode>(topic);
  }

  virtual ~TestSubscriber() {}

  bool has_message_ready() {
    usleep(1000);
    rclcpp::spin_some(this->node);
    return this->node->has_message_ready();
  }

  std::shared_ptr<MsgType> get_message() {
    usleep(1000);
    rclcpp::spin_some(this->node);
    return this->node->get_message();
  }

private:
  class TestSubscriberNode : public rclcpp::Node {
  public:
    TestSubscriberNode(std::string topic) : Node("test_subscriber_node_" + topic) {
      this->received_messages = std::make_unique<std::queue<std::shared_ptr<MsgType>>>();
      this->subscription = this->create_subscription<MsgType>(topic, 64, std::bind(
	& TestSubscriberNode::receive_message, this, std::placeholders::_1));
    }

    virtual ~TestSubscriberNode() {}

    bool has_message_ready() {
      return ! received_messages->empty();
    }

    std::shared_ptr<MsgType> get_message() {
      std::shared_ptr<MsgType> message = received_messages->front();
      received_messages->pop();
      return message;
    }

  private:
    void receive_message(const std::shared_ptr<MsgType> message) {
      received_messages->push(message);
    }

    std::unique_ptr<std::queue<std::shared_ptr<MsgType>>> received_messages;
    typename rclcpp::Subscription<MsgType>::SharedPtr subscription;
  };

  std::shared_ptr<TestSubscriberNode> node;
};

}
}
