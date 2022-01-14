#include <gtest/gtest.h>
#include <memory>
#include "nova_gps/types.hpp"

using namespace Nova;


// Test the direct constructor and reads
TEST(TestBuffer, initialize_from_data) {
  ByteBuffer buf(8, std::unique_ptr<uint8_t[]>(new uint8_t[8]{1, 2, 3, 4, 0xEF, 0xBE, 0xAD, 0xDE}));
  ASSERT_EQ(buf.read_byte(), 1);
  ASSERT_EQ(buf.read_byte(), 2);
  ASSERT_EQ(buf.read_word(), (4 << 8) | 3);
  ASSERT_EQ(buf.read_dword(), 0xDEADBEEF);
}

TEST(TestBuffer, out_of_bounds) {
  ByteBuffer buf(5, std::unique_ptr<uint8_t[]>(new uint8_t[5]{1, 2, 3, 4, 5}));
  buf.read_dword();
  ASSERT_DEBUG_DEATH(buf.read_dword(), "");
}

TEST(TestBuffer, empty) {
  ByteBuffer buf(99);
  buf.seek(0, SEEK_END);
  ASSERT_EQ(buf.read_byte(), 0);
}

TEST(TestBuffer, seek) {
  ByteBuffer buf(6, std::unique_ptr<uint8_t[]>(new uint8_t[6]{1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(buf.get_pos(), 0);
  ASSERT_EQ(buf.seek(0, SEEK_CUR), 0);
  ASSERT_EQ(buf.seek(3, SEEK_CUR), 3);
  ASSERT_EQ(buf.seek(-1, SEEK_CUR), 2);
  ASSERT_EQ(buf.seek(5, SEEK_SET), 5);
  ASSERT_EQ(buf.seek(-1, SEEK_END), 4);
}