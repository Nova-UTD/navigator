#include <gtest/gtest.h>
#include <memory>
#include "nova_gps/types.hpp"
#include "nova_gps/UBX.hpp"
using namespace Nova;


TEST(TestUBX, read_ack) {
  ByteBuffer buf(10, std::unique_ptr<uint8_t[]>(new uint8_t[10]{0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0xEF, 0xBE, 0xB4, 0xB5}));
  auto msgl = Nova::UBX::parse_ubx_messages(buf);
  ASSERT_EQ((*msgl).size(), 1u);
  auto msg = std::move((*msgl)[0]);
  ASSERT_EQ(msg->mclass, 0x05);
  ASSERT_EQ(msg->id, 0x01);
  ASSERT_EQ(msg->checksum, 0xB5B4);
  ASSERT_EQ(msg->data->read_word(), 0xBEEF);
}

TEST(TestUBX, read_two) {
  ByteBuffer buf(20, std::unique_ptr<uint8_t[]>(new uint8_t[20]{0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0xEF, 0xBE, 0xB4, 0xB5,
                                                                0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0xEF, 0xBE, 0xB4, 0xB5}));
  auto msgl = Nova::UBX::parse_ubx_messages(buf);
  ASSERT_EQ((*msgl).size(), 2u);
}

TEST(TestUBX, read_too_big) {
  ByteBuffer buf(10, std::unique_ptr<uint8_t[]>(new uint8_t[10]{0xB5, 0x62, 0x05, 0x01, 0x02, 0x40, 0xEF, 0xBE, 0xB4, 0xB5}));
  ASSERT_ANY_THROW(Nova::UBX::parse_ubx_messages(buf));
}

TEST(TestUBX, bad_checksum) {
  ByteBuffer buf(10, std::unique_ptr<uint8_t[]>(new uint8_t[10]{0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0xEF, 0xBE, 0xAA, 0xAB}));
  ASSERT_ANY_THROW(Nova::UBX::parse_ubx_messages(buf));
}