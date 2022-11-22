/*
 * Package:   voltron_test_utils
 * Filename:  test_test_client_server.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Yes, we are going to test the testing tools. And yes, we will use
// other testing tools to accomplish this.

#include <gtest/gtest.h> // Testing framework
#include "rclcpp/rclcpp.hpp" // For init() and shutdown()
#include "std_srvs/srv/set_bool.hpp" // A test message to use
#include "voltron_test_utils/TestClient.hpp" // Our test client
#include "voltron_test_utils/TestServer.hpp" // Our test server


using namespace Voltron::TestUtils;

class TestTestClientServer : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    std_srvs::srv::SetBool::Response default_response;
    default_response.success = false;
    default_response.message = "Default response";
    this->test_server = std::make_unique<TestServer<std_srvs::srv::SetBool>>("test_topic", default_response);
    this->test_client = std::make_unique<TestClient<std_srvs::srv::SetBool>>("test_topic");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::unique_ptr<TestServer<std_srvs::srv::SetBool>> test_server;
  std::unique_ptr<TestClient<std_srvs::srv::SetBool>> test_client;
};

TEST_F(TestTestClientServer, test_initializes) {
  SUCCEED(); // Succeeds if the SetUp() function runs successfully
}

TEST_F(TestTestClientServer, test_simple_request) {
  ASSERT_FALSE(this->test_client->request_complete());
  ASSERT_FALSE(this->test_server->has_received_request());
  std_srvs::srv::SetBool::Response response_to_send;
  response_to_send.success = true;
  this->test_server->enqueue_response(response_to_send);
  std_srvs::srv::SetBool::Request request_to_send;
  request_to_send.data = false;
  this->test_client->send_request(request_to_send);
  ASSERT_TRUE(this->test_server->has_received_request());
  std_srvs::srv::SetBool::Request received_request = this->test_server->get_received_request();
  ASSERT_EQ(received_request.data, false);
  ASSERT_TRUE(this->test_client->request_complete());
  std_srvs::srv::SetBool::Response received_response = this->test_client->get_response();
  ASSERT_EQ(received_response.success, true);
}
