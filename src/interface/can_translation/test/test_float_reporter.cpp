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

#include "nova_msgs/msg/can_frame.hpp" // CAN messages
#include "voltron_test_utils/TestSubscriber.hpp"
#include "voltron_test_utils/TestPublisher.hpp"

#include "can_translation/FloatReporterNode.hpp"

using namespace Voltron::TestUtils;
using namespace navigator::can_translation;

class TestFloatReporter : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    angle_subscription = std::make_unique<TestSubscriber<
      std_msgs::msg::Float32>>("can_translation_result_topic");
    can_publisher = std::make_unique<TestPublisher<
      nova_msgs::msg::CanFrame>>("can_translation_incoming_can_frames");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::unique_ptr<TestSubscriber<std_msgs::msg::Float32>> angle_subscription;
  std::unique_ptr<TestPublisher<nova_msgs::msg::CanFrame>> can_publisher;
};

// Test that the node initializes and comes online properly
TEST_F(TestFloatReporter, test_initializes) {
  // All the necessary code for this test is already written in the
  // fixture
}

// Test that the node only responds to messages with the correct identifier
TEST_F(TestFloatReporter, test_responds_to_correct_id) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 65535, 0x001, 0, 64, });

  auto message_1 = nova_msgs::msg::CanFrame();
  message_1.data = 0x0000000000004E20;
  message_1.identifier = 0x002;
  can_publisher->send_message(message_1);

  auto message_2 = nova_msgs::msg::CanFrame();
  message_2.data = 0x0000000000004E20;
  message_2.identifier = 0x000;
  can_publisher->send_message(message_2);

  rclcpp::spin_some(my_reporter);
  rclcpp::spin_some(my_reporter);
  ASSERT_FALSE(angle_subscription->has_message_ready());

  auto message_3 = nova_msgs::msg::CanFrame();
  message_3.data = 0x0000000000004E20;
  message_3.identifier = 0x001;
  can_publisher->send_message(message_3);

  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
}

// Test basic function of sending a message without any refinement or scaling
TEST_F(TestFloatReporter, test_whole_field_unscaled) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 65535, 0x001, 0, 64, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x0000000000004E20; // Should be 20000
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 20000);
}

// Test that we can knock extra data off the end
TEST_F(TestFloatReporter, test_small_field_at_end_unscaled) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 65535, 0x001, 0, 16, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B98253ECE4E20; // Should be 20000
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 20000);
}

// Test that we can knock extra data off the beginning
TEST_F(TestFloatReporter, test_small_field_in_middle_unscaled) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 65535, 0x001, 16, 16, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B98254E203ECE; // Should be 20000
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 20000);
}

// Test that we can perform simple scaling on the value
TEST_F(TestFloatReporter, test_small_field_at_end_scaled_simple) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 1, 0x001, 0, 16, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B98254E203333; // Should be 13107, or 1/5 of the max
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 0.2);
}

// Test that we can perform scaling and retrieve a field from the middle
TEST_F(TestFloatReporter, test_small_field_in_middle_scaled_simple) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 65535, 0, 1, 0x001, 16, 16, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B982533334E20; // Should be 13107, or 1/5 of the max
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 0.2);
}

// Test that we can perform more complex scaling
TEST_F(TestFloatReporter, test_scales_complex_1) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 128, 153, -1, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B982533338A20; // Should be 138, which is 40% of the way between our values, yielding -0.2
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, -0.2);
}

TEST_F(TestFloatReporter, test_scales_complex_2) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 60000, 10, 14.8, 0x001, 8, 16, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B9825333A9820; // Should be 15000, which is 25% of the way between our values, yielding 11.2
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 11.2);
}

// Test that we correctly handle scaling at the extremes
TEST_F(TestFloatReporter, test_scales_at_min) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 128, 153, -1, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B982533338020; // Should be 128, our minimum value
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, -1);
}

TEST_F(TestFloatReporter, test_scales_at_max) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 128, 153, -1, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B982533339920; // Should be 153, our maximum value
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 1);
}

// Finally, test that we correctly extrapolate beyond the extremes
TEST_F(TestFloatReporter, test_extrapolates_beyond_min) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 128, 153, -1, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B982533336720; // Should be 103
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, -3);
}

TEST_F(TestFloatReporter, test_extrapolates_beyond_max) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 128, 153, -1, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0x083B98253333B220; // Should be 178
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 3);
}

// Test that we can handle the extreme case of the data field having maximum value
TEST_F(TestFloatReporter, test_handles_max_data) {
  auto my_reporter = std::make_shared<FloatReporterNode>(
    float_reporter_params { 0, 255, 0, 1, 0x001, 8, 8, });
  auto message_to_send = nova_msgs::msg::CanFrame();
  message_to_send.data = 0xFFFFFFFFFFFFFFFF; // Should be 255, our maximum value
  message_to_send.identifier = 0x001;
  can_publisher->send_message(message_to_send);
  rclcpp::spin_some(my_reporter);
  ASSERT_TRUE(angle_subscription->has_message_ready());
  EXPECT_FLOAT_EQ(angle_subscription->get_message()->data, 1);
}
