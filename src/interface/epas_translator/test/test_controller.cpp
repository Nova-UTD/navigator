/*
 * Package:   epas_translator
 * Filename:  test_controller.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

// Test only the controller node in isolation

#include <cmath>
#include <gtest/gtest.h> // Testing framework
#include <iostream>
#include <iomanip>
#include <memory> // std::make_shared
#include <unistd.h> // sleep

#include "rclcpp/rclcpp.hpp" // To control the node
#include "std_msgs/msg/float32.hpp"

#include "nova_msgs/msg/can_frame.hpp" // CAN messages
#include "voltron_test_utils/TestSubscriber.hpp"// Test subscriber
#include "voltron_test_utils/TestPublisher.hpp" // Test publisher

#include "epas_translator/ControllerNode.hpp"

using namespace Voltron::TestUtils;
using namespace Voltron::EpasSteering;

constexpr int can_message_3_identifier = 0x296;

class TestController : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_controller = std::make_shared<ControllerNode>();
    can_subscription = std::make_unique<TestSubscriber<
      nova_msgs::msg::CanFrame>>("epas_translator_outgoing_can_frames");
    power_publisher = std::make_unique<TestPublisher<
      std_msgs::msg::Float32>>("epas_translator_steering_power");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<ControllerNode> my_controller;
  std::unique_ptr<TestSubscriber<nova_msgs::msg::CanFrame>> can_subscription;
  std::unique_ptr<TestPublisher<std_msgs::msg::Float32>> power_publisher;
};

TEST_F(TestController, test_initializes) {
  // All the necessary code for this test is already written in the
  // fixture
}

TEST_F(TestController, test_sends_default_messages) {
  usleep(40000); // About 40ms, enough that we must have sent at least one message
  rclcpp::spin_some(my_controller);
  ASSERT_TRUE(can_subscription->has_message_ready());
  auto received_message = can_subscription->get_message();
  ASSERT_EQ(received_message->identifier, can_message_3_identifier);
  ASSERT_EQ(received_message->data, 0x807F05u);
}

TEST_F(TestController, test_change_power) {
  std_msgs::msg::Float32 pub_message;
  pub_message.data = -1; // Set power to -1, which just means left 100%
  power_publisher->send_message(pub_message);

  bool test_passed = false;
  for(int i = 0; i < 8; i++) {
    usleep(25000);
    rclcpp::spin_some(my_controller);
    ASSERT_TRUE(can_subscription->has_message_ready());
    auto received_message = can_subscription->get_message();
    if(received_message->data == 0xFF0005) {
      test_passed = true;
      break;
    }
  }
  ASSERT_TRUE(test_passed);
}

TEST_F(TestController, test_clamps) {
  std_msgs::msg::Float32 pub_message;
  pub_message.data = -1.5;
  power_publisher->send_message(pub_message);

  bool test_passed = false;
  for(int i = 0; i < 8; i++) {
    usleep(25000);
    rclcpp::spin_some(my_controller);
    ASSERT_TRUE(can_subscription->has_message_ready());
    auto received_message = can_subscription->get_message();
    if(received_message->data == 0xFF0005) {
      test_passed = true;
      break;
    }
  }
  ASSERT_TRUE(test_passed);
}
