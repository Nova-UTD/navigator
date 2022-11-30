/*
 * Package:   voltron_test_utils
 * Filename:  TestServer.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <memory>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <unistd.h>

namespace Voltron {
namespace TestUtils {

template <typename ServiceType> class TestServer {
public:
  typedef typename ServiceType::Request RequestType;
  typedef typename ServiceType::Response ResponseType;

  TestServer(std::string topic, ResponseType default_response) {
    this->node = std::make_shared<TestServerNode>(topic, default_response);
  }

  void enqueue_response(ResponseType response) {
    this->node->enqueue_response(response);
  }

  bool has_received_request() {
    usleep(1000);
    rclcpp::spin_some(this->node);
    return this->node->has_received_request();
  }

  RequestType get_received_request() {
    usleep(1000);
    rclcpp::spin_some(this->node);
    return this->node->get_received_request();
  }

private:
  class TestServerNode : public rclcpp::Node {
  public:
    TestServerNode(std::string topic, ResponseType default_response) : Node("test_server_node_" + topic) {
      this->server = this->create_service<ServiceType>(topic,
	[&] (const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
	  this->handle_request(request, response);
        }
      );
      this->default_response = default_response;
    }

    void enqueue_response(ResponseType response) {
      this->enqueued_responses.push(response);
    }

    bool has_received_request() {
      return ! this->received_requests.empty();
    }

    RequestType get_received_request() {
      RequestType value  = this->received_requests.front();
      this->received_requests.pop();
      return value;
    }

  private:
    void handle_request(const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
      this->received_requests.push(*request);
      if(this->enqueued_responses.empty()) {
	*response = default_response;
      } else {
	*response = enqueued_responses.front();
	enqueued_responses.pop();
      }
    }

    std::queue<RequestType> received_requests;
    std::queue<ResponseType> enqueued_responses;
    std::shared_ptr<rclcpp::Service<ServiceType>> server;
    ResponseType default_response;
  };

  std::shared_ptr<TestServerNode> node;
};
  
}
}
