/*
 * Package:   can_translation
 * Filename:  test/TestFloatReporterNode.cpp
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

#include "voltron_msgs/msg/can_frame.hpp" // CAN messages
#include "voltron_test_utils/TestSubscriber.hpp"
#include "voltron_test_utils/TestPublisher.hpp"

#include "can_translation/FloatReporterNode.hpp"

using namespace Voltron::TestUtils;
using namespace navigator::can_translation;

constexpr int can_message_2_identifier = 0x292;

class TestFloatReporter : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_reporter = std::make_shared<FloatReporterNode>(float_reporter_params {
						       40,
						       80,
						       -0.5,
						       0.5,
						       0x292,
						       0,
						       8,
	});
    angle_subscription = std::make_unique<TestSubscriber<
      std_msgs::msg::Float32>>("result_topic");
    can_publisher = std::make_unique<TestPublisher<
      voltron_msgs::msg::CanFrame>>("incoming_can_frames");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<FloatReporterNode> my_reporter;
  std::unique_ptr<TestSubscriber<std_msgs::msg::Float32>> angle_subscription;
  std::unique_ptr<TestPublisher<voltron_msgs::msg::CanFrame>> can_publisher;
};

TEST_F(TestFloatReporter, test_initializes) {
  // All the necessary code for this test is already written in the
  // fixture
}

TEST_F(TestFloatReporter, test_reports_once) {
  auto message_to_send = voltron_msgs::msg::CanFrame();
  message_to_send.data = 0x0000000000000046; // Should be 70, 75% of the maximum
  message_to_send.identifier = can_message_2_identifier;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  auto received_message = angle_subscription->get_message();
  const float target = 0.25;
  float error = received_message->data - target;
  error = std::abs(error);
  ASSERT_LT(error, 0.01);
}
