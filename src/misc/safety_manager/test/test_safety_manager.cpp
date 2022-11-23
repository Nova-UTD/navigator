/*
 * Package:   safety_manager
 * Filename:  test_safety_manager.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Basic tests for the safety manager

#include <gtest/gtest.h>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/srv/safety_event.hpp"

#include "voltron_test_utils/TestClient.hpp"

#include "safety_manager/SafetyManagerNode.hpp"

using namespace Voltron::TestUtils;
using namespace Nova::SafetyManager;

class TestSafetyManager : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_safety_manager = std::make_shared<SafetyManagerNode>();
    test_client = std::make_unique<TestClient<voltron_msgs::srv::SafetyEvent>>("safety_events");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<SafetyManagerNode> my_safety_manager;
  std::unique_ptr<TestClient<voltron_msgs::srv::SafetyEvent>> test_client;
};

TEST_F(TestSafetyManager, test_initializes) {
  // All the ecessary code for this test is already written in the
  // fixture
}

// Not much we can do here until actual logic is implemented
TEST_F(TestSafetyManager, test_event) {
  auto message_to_send = voltron_msgs::srv::SafetyEvent::Request();
  message_to_send.event_uid = 4612894;
  message_to_send.sequence_number = 0;
  message_to_send.status = voltron_msgs::srv::SafetyEvent::Request::STATUS_RESOLVED;
  message_to_send.description = "A test event";
  message_to_send.additional_data = "";
  test_client->send_request(message_to_send);
  usleep(1000);
  rclcpp::spin_some(my_safety_manager);
  ASSERT_TRUE(test_client->request_complete());
}
