/*
 * Package:   voltron_test_utils
 * Filename:  test_test_publisher_subscriber.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// Yes, we are going to test the testing tools. And yes, we will use
// other testing tools to accomplish this.

#include <gtest/gtest.h> // Testing framework
#include <memory> // std::make_unique
#include "rclcpp/rclcpp.hpp" // For init() and shutdown()
#include "std_msgs/msg/int64.hpp" // A test message to use
#include "voltron_test_utils/TestSubscriber.hpp" // Our test subscriber
#include "voltron_test_utils/TestPublisher.hpp" // Our test publisher

using namespace Voltron::TestUtils;

class TestTestPublisherSubscriber : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    test_subscriber = std::make_unique<TestSubscriber<std_msgs::msg::Int64>>("test_topic");
    test_publisher = std::make_unique<TestPublisher<std_msgs::msg::Int64>>("test_topic");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::unique_ptr<TestSubscriber<std_msgs::msg::Int64>> test_subscriber;
  std::unique_ptr<TestPublisher<std_msgs::msg::Int64>> test_publisher;
};
    

TEST_F(TestTestPublisherSubscriber, test_initializes) {
  SUCCEED(); // Succeeds if the SetUp() function runs successfully
}

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
