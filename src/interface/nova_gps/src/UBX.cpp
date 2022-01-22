#include "nova_gps/types.hpp"
#include "nova_gps/UBX.hpp"
#include "string.h"
using namespace Nova::UBX;
std::unique_ptr<std::vector<std::unique_ptr<UBXMessage>>> Nova::UBX::parse_ubx_messages(Nova::ByteBuffer & buf) {
  auto split = std::make_unique<std::vector<std::unique_ptr<UBXMessage>>>();
  std::size_t marker_data;
  while(buf.get_pos() < buf.size()) {
    auto first = buf.read_byte();
    auto second = buf.read_byte();
    if(first != 0xB5 && second != 0x62) {
      throw std::runtime_error("Didn't read sync characters first");
    }
    auto mclass = buf.read_byte();
    auto id = buf.read_byte();
    auto length = buf.read_word();
    marker_data = buf.get_pos();
    if(length + marker_data > buf.size()) {
      throw std::runtime_error("Possible block segmentation");
    }
    std::unique_ptr<Nova::ByteBuffer> data_buffer = std::make_unique<Nova::ByteBuffer>(length);
    memcpy(&(*data_buffer)[0], &buf[buf.get_pos()], length);
    buf.seek(length, SEEK_CUR);
    auto read_checksum = buf.read_word();
    auto msg = std::make_unique<UBXMessage>();
    msg->mclass = mclass;
    msg->id = id;
    msg->checksum = read_checksum;
    msg->data = std::move(data_buffer);
    auto calculated_checksum = calculate_checksum(*msg);
    if(calculated_checksum != read_checksum) {
      throw std::runtime_error("Checksum mismatch");
    }
    split->push_back(std::move(msg));
  }
  return split;
}
uint16_t Nova::UBX::calculate_checksum(Nova::UBX::UBXMessage & message) {
  uint16_t CB_A = message.mclass, CB_B = message.mclass;
  CB_A = (CB_A + message.id) % 255;
  CB_B = (CB_B + CB_A) % 255;
  auto size = message.data->size();
  for(uint32_t i = 0; i < size; ++i) {
    CB_A = (CB_A + message.data->operator[](i)) % 255;
    CB_B = (CB_B + CB_A) % 255;
  }
  return CB_A | (CB_B << 8);
}

std::unique_ptr<Nova::UBX::UBXMessage> Nova::UBX::set_message_rate(uint8_t message_class, uint8_t message_id, uint8_t rate) {
  auto buf = std::make_unique<Nova::ByteBuffer>(8);
  auto msg = std::make_unique<Nova::UBX::UBXMessage>();
  (*buf)[0] = message_class;
  (*buf)[1] = message_id;
  (*buf)[2] = 0;
  (*buf)[3] = 0;
  (*buf)[4] = 0;
  (*buf)[5] = 0;
  (*buf)[6] = rate;
  (*buf)[7] = 0;
  msg->mclass = 0x05u;
  msg->id = 0x01u;
  msg->data = std::move(buf);
  msg->checksum = calculate_checksum(*msg);
  return msg;
}