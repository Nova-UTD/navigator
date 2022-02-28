/*
 * Package:   msg_translation
 * Filename:  test/TestVelocityToTwistNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <cmath> // abs
#include <gtest/gtest.h> // Testing framework
#include <memory> // std::make_shared

#include "rclcpp/rclcpp.hpp" // To control the node
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

#include "voltron_test_utils/TestSubscriber.hpp"
#include "voltron_test_utils/TestPublisher.hpp"

#include "msg_translation/VelocityToTwistNode.hpp"

using namespace Voltron::TestUtils;
using namespace navigator::msg_translation;

class TestVelocityToTwist : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_translator = std::make_shared<VelocityToTwistNode>();
    twist_subscription = std::make_unique<TestSubscriber<
      geometry_msgs::msg::TwistWithCovariance>>("twist_topic");
    velocity_publisher = std::make_unique<TestPublisher<
      std_msgs::msg::Float32>>("velocity_topic");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<VelocityToTwistNode> my_translator;
  std::unique_ptr<TestSubscriber<geometry_msgs::msg::TwistWithCovariance>> twist_subscription;
  std::unique_ptr<TestPublisher<std_msgs::msg::Float32>> velocity_publisher;
};

TEST_F(TestVelocityToTwist, test_initializes) {
  // All the necessary code for this test is already written in the
  // fixture
}

TEST_F(TestVelocityToTwist, test_converts_once) {
  auto message_to_send = std_msgs::msg::Float32();
  message_to_send.data = 8.94;
  velocity_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_translator);
  ASSERT_TRUE(twist_subscription->has_message_ready());
  auto received_message = twist_subscription->get_message();
  float error = received_message->twist.linear.x - 8.94;
  error = std::abs(error);
  ASSERT_LT(error, 0.01);
}
