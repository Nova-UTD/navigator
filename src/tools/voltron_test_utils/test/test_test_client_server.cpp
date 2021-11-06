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
#include "std_srvs/srv/empty.hpp" // A test message to use
#include "voltron_test_utils/TestClient.hpp" // Our test client
#include "voltron_test_utils/TestServer.hpp" // Our test server


using namespace Voltron::TestUtils;

class TestTestClientServer : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    std_srvs::srv::Empty::Response default_response;
    this->test_server = std::make_unique<TestServer<std_srvs::srv::Empty>>("test_topic", default_response);
    this->test_client = std::make_unique<TestClient<std_srvs::srv::Empty>>("test_topic");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::unique_ptr<TestServer<std_srvs::srv::Empty>> test_server;
  std::unique_ptr<TestClient<std_srvs::srv::Empty>> test_client;
};

TEST_F(TestTestClientServer, test_initializes) {
  SUCCEED(); // Succeeds if the SetUp() function runs successfully
}

/*
TEST_F(TestTestPublisherSubscriber, test_simple_communication) {
  ASSERT_FALSE(this->test_subscriber->has_message_ready());
  auto message_to_send = std_msgs::msg::Int64();
  message_to_send.data = 123456;
  this->test_publisher->send_message(message_to_send);
  ASSERT_TRUE(this->test_subscriber->has_message_ready());
  auto received_message = this->test_subscriber->get_message();
  ASSERT_EQ(received_message->data, 123456);
  ASSERT_FALSE(this->test_subscriber->has_message_ready());
}
*/
