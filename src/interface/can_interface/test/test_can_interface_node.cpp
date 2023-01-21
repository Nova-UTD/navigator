/*
 * Package:   can_interface
 * Filename:  test_interface.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// Test the ROS node to bridge the CAN bus and ROS. Note that for
// these tests to pass, you need to have a CAN bus named vcan0
// operational on your system.

#include <gtest/gtest.h> // Testing framework
#include "rclcpp/rclcpp.hpp"
#include <unistd.h>

#include "voltron_test_utils/TestPublisher.hpp"
#include "voltron_test_utils/TestSubscriber.hpp"
#include "nova_msgs/msg/can_frame.hpp"
#include "can_interface/CanBus.hpp"
#include "can_interface/CanFrame.hpp"

#include "can_interface/CanInterfaceNode.hpp" // The class we are testing

using namespace navigator::can_interface;
using namespace Voltron::TestUtils;

class TestCanInterfaceNode : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    interface_node = std::make_shared<CanInterfaceNode>("vcan0");
    can_bus = std::make_unique<CanBus>("vcan0");
    can_publisher = std::make_unique<TestPublisher<
      nova_msgs::msg::CanFrame>>("can_interface_outgoing_can_frames");
    can_subscriber = std::make_unique<TestSubscriber<
      nova_msgs::msg::CanFrame>>("can_interface_incoming_can_frames");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<CanInterfaceNode> interface_node;
  std::unique_ptr<CanBus> can_bus;
  std::unique_ptr<TestPublisher<nova_msgs::msg::CanFrame>> can_publisher;
  std::unique_ptr<TestSubscriber<nova_msgs::msg::CanFrame>> can_subscriber;
};

TEST_F(TestCanInterfaceNode, test_initializes) {
  SUCCEED();
}

TEST_F(TestCanInterfaceNode, test_incoming_frame) {
  auto frame = navigator::can_interface::CanFrame(0x123, 0x12345678);
  this->can_bus->write_frame(frame);
  usleep(20000); // 20ms, enough that we should receive frames on next spin
  rclcpp::spin_some(this->interface_node);
  ASSERT_TRUE(this->can_subscriber->has_message_ready());
  nova_msgs::msg::CanFrame received_message = *(this->can_subscriber->get_message());
  ASSERT_EQ(received_message.identifier, 0x123u);
  ASSERT_EQ(received_message.data, 0x12345678u);
}

TEST_F(TestCanInterfaceNode, test_outgoing_frame) {
  nova_msgs::msg::CanFrame frame_message;
  frame_message.identifier = 0x123;
  frame_message.data = 0x12345678;
  this->can_publisher->send_message(frame_message);
  rclcpp::spin_some(this->interface_node);
  ASSERT_TRUE(this->can_bus->is_frame_ready());
  std::unique_ptr<CanFrame> received_message = this->can_bus->read_frame();
  ASSERT_EQ(received_message->get_identifier(), 0x123u);
  ASSERT_EQ(received_message->get_data(), 0x12345678u);
}
