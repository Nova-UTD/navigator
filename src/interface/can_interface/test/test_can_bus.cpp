/*
 * Package:   can_interface
 * Filename:  test_can_bus.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// Test the CanBus class (isolated from the ROS node). Note that for
// these tests to pass, you need to have a CAN bus named vcan0
// operational on your system.

#include <gtest/gtest.h> // Testing framework
#include <memory> // std::unique_ptr

#include "can_interface/CanBus.hpp" // The class we are testing, obviously
#include "can_interface/CanFrame.hpp" // Needed to interact with CanBus

using namespace navigator::can_interface;

TEST(TestCanBus, test_intiializes) {
  CanBus my_bus("vcan0"); // If this passes, the test succeeds
}

// Just testing for the general ability of these busses to communicate
// with each other
TEST(TestCanBus, test_communication) {
  CanBus bus1("vcan0");
  CanBus bus2("vcan0");
  ASSERT_FALSE(bus1.is_frame_ready());
  ASSERT_FALSE(bus2.is_frame_ready());
  CanFrame my_frame(0x292, 0x1234567890123456u);
  bus1.write_frame(my_frame);
  ASSERT_FALSE(bus1.is_frame_ready());
  ASSERT_TRUE(bus2.is_frame_ready());
  std::unique_ptr<CanFrame> received_frame = bus2.read_frame();
  ASSERT_EQ(received_frame->get_identifier(), 0x292u);
  ASSERT_EQ(received_frame->get_data(), 0x1234567890123456u);
  ASSERT_FALSE(bus2.is_frame_ready());
  CanFrame new_frame(0x293, 0x1234567890123457);
  bus2.write_frame(my_frame);
  bus2.write_frame(new_frame);
  ASSERT_TRUE(bus1.is_frame_ready());
  ASSERT_FALSE(bus2.is_frame_ready());
  std::unique_ptr<CanFrame> received_frame_1 = bus1.read_frame();
  ASSERT_TRUE(bus1.is_frame_ready());
  std::unique_ptr<CanFrame> received_frame_2 = bus1.read_frame();
  ASSERT_FALSE(bus1.is_frame_ready());
  ASSERT_EQ(received_frame_1->get_identifier(), 0x292u);
  ASSERT_EQ(received_frame_1->get_data(), 0x1234567890123456u);
  ASSERT_EQ(received_frame_2->get_identifier(), 0x293u);
  ASSERT_EQ(received_frame_2->get_data(), 0x1234567890123457u);
}
