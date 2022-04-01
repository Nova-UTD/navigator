/*
 * Package:   can_interface
 * Filename:  test_can_frame.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// Test the CanFrame class (isolated from the actual CanBus class).

#include <gtest/gtest.h> // Testing framework
#include <linux/can.h> // struct can_frame
#include <memory> // std::unique_ptr

#include "can_interface/CanFrame.hpp"

using namespace navigator::can_interface;

// Test the direct constructor
TEST(TestCanFrame, initialize_from_data) {
  CanFrame my_frame(0x292, 0x1234567890123456);
  ASSERT_EQ(my_frame.get_identifier(), 0x292u);
  ASSERT_EQ(my_frame.get_data(), 0x1234567890123456u);
}

// Test the constructor that copies from a can_frame struct
TEST(TestCanFrame, initialize_from_system_frame) {
  struct can_frame system_frame;
  system_frame.can_id = 0x292;
  *((CanFrame::data_t *) system_frame.data) = 0x0102030405060708;
  CanFrame my_frame(system_frame);
  ASSERT_EQ(my_frame.get_identifier(), 0x292u);
  ASSERT_EQ(my_frame.get_data(), 0x0102030405060708u);
}

// Test converting our representation to a can_frame struct
TEST(TestCanFrame, convert_to_system_frame) {
  CanFrame my_frame(0x292, 0x0102030405060708);
  std::unique_ptr<struct can_frame> system_frame = my_frame.to_system_frame();
  ASSERT_EQ(system_frame->can_id, 0x292u);
  ASSERT_EQ(*((CanFrame::data_t*) system_frame->data), (CanFrame::data_t) 0x0102030405060708);
  ASSERT_EQ(system_frame->can_dlc, 8);
}
