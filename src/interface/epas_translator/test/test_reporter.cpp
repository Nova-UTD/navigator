/*
 * Package:   epas_translator
 * Filename:  test_reporter.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// Test only the reporter node in isolation

#include <cmath> // abs
#include <gtest/gtest.h> // Testing framework
#include <memory> // std::make_shared

#include "rclcpp/rclcpp.hpp" // To control the node
#include "std_msgs/msg/float32.hpp"

#include "nova_msgs/msg/can_frame.hpp" // CAN messages
#include "voltron_test_utils/TestSubscriber.hpp" // Our test subscriber
#include "voltron_test_utils/TestPublisher.hpp" // Our test publisher

#include "epas_translator/ReporterNode.hpp"

using namespace Voltron::TestUtils;
using namespace Voltron::EpasSteering;

constexpr int can_message_2_identifier = 0x292;

class TestReporter : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_reporter = std::make_shared<ReporterNode>(49, 71);
    angle_subscription = std::make_unique<TestSubscriber<
      std_msgs::msg::Float32>>("epas_translator_real_steering_angle");
    can_publisher = std::make_unique<TestPublisher<
      nova_msgs::msg::CanFrame>>("epas_translator_incoming_can_frames");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<ReporterNode> my_reporter;
  std::unique_ptr<TestSubscriber<std_msgs::msg::Float32>> angle_subscription;
  std::unique_ptr<TestPublisher<nova_msgs::msg::CanFrame>> can_publisher;
};

TEST_F(TestReporter, test_initializes) {
  // All the necessary code for this test is already written in the
  // fixture
}

TEST_F(TestReporter, test_reports_once) {
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x000000000000003F;
  message_to_send.identifier = can_message_2_identifier;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  auto received_message = angle_subscription->get_message();
  const float target = 0.153164627;
  float error = received_message->data - target;
  error = std::abs(error);
  ASSERT_LT(error, 0.01);
}
