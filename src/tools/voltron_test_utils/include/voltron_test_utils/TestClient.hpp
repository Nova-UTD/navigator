/*
 * Package:   voltron_test_utils
 * Filename:  TestClient.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <chrono>
#include <future>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <unistd.h>

using namespace std::chrono_literals;

namespace Voltron {
namespace TestUtils {

template <typename ServiceType> class TestClient {
public:
  typedef typename ServiceType::Request RequestType;
  typedef typename ServiceType::Response ResponseType;

  TestClient(std::string topic) {
    this->node = std::make_shared<TestClientNode>(topic);
  }

  void send_request(RequestType request) {
    this->current_response = this->node->send_request(request);
    rclcpp::spin_some(this->node);
  }

  bool request_complete() {
    if(! this->current_response.valid()) return false;
    usleep(1000);
    rclcpp::spin_some(this->node);
    return this->current_response.wait_for(0s) == std::future_status::ready;
  }

  ResponseType get_response() {
    rclcpp::spin_until_future_complete(this->node, this->current_response);
    return *(this->current_response.get());
  }

private:
  typedef rclcpp::Client<ServiceType> ClientType;
  
  class TestClientNode : public rclcpp::Node {
  public:
    TestClientNode(std::string topic) : Node("test_client_node_" + topic) {
      this->client = this->create_client<ServiceType>(topic);
    }

    typename ClientType::SharedFuture send_request(RequestType request) {
      return this->client->async_send_request(std::make_shared<RequestType>(request));
    }

  private:
    std::shared_ptr<ClientType> client;
  };

  typename ClientType::SharedFuture current_response;
  std::shared_ptr<TestClientNode> node;
};

}
}
