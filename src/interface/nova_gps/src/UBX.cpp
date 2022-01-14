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
    auto checksum = buf.read_word();
    auto msg = std::make_unique<UBXMessage>();
    msg->mclass = mclass;
    msg->id = id;
    msg->checksum = checksum;
    msg->data = std::move(data_buffer);
    split->push_back(std::move(msg));
  }
  return split;
}