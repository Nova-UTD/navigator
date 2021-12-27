/*
 * Package:   bridge_manager
 * Filename:  test_bridge_manager.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// TBD testing for Bridging Manager

#include <gtest/gtest.h>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/srv/safety_event.hpp"

#include "voltron_test_utils/TestClient.hpp"

#include "bridge_manager/BridgeManagerNode.hpp"

using namespace Voltron::TestUtils;
using namespace Nova;

class TestBridgeManager : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    my_bridge_manager = std::make_shared<BridgeManagerNode>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<BridgeManagerNode> my_bridge_manager;
};

TEST_F(TestBridgeManager, test_initializes) {
  // TBD
}

// Need to finish this in the future
TEST_F(TestBridgeManager, test_event) {
  usleep(1000);
  rclcpp::spin_some(my_bridge_manager);
}
