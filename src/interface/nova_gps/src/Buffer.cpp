#include "nova_gps/types.hpp"
#include <cassert>
using namespace Nova;
template <typename T>
T& Buffer<T>::operator[] (unsigned int idx) {
  assert(idx >= 0 && idx < bufsize);
  return data[idx];
}
template <typename T>
const T& Buffer<T>::operator[] (unsigned int idx) const {
  assert(idx >= 0 && idx < bufsize);
  return data[idx];
}
template <typename T>
Buffer<T>::Buffer(std::size_t size) {
  this->bufsize = size;
  // std::make_unique forces us to initialize our memory, which is an undesirable overhead
  // considering we overwrite the buffer immediately in nearly all cases
  this->data = std::unique_ptr<T[]>(new T[size]);
}
template <typename T>
Buffer<T>::Buffer(std::size_t size, std::unique_ptr<T[]> data) {
  this->bufsize = size;
  this->data = std::move(data);
}

uint8_t ByteBuffer::read_byte() {
  return this->operator[](this->pointer++);
}

uint16_t ByteBuffer::read_word() {
  uint16_t low = read_byte();
  uint16_t hi  = read_byte();
  return (hi << 8) | low;
}

uint32_t ByteBuffer::read_dword() {
  uint32_t low = read_word();
  uint32_t hi  = read_word();
  return (hi << 16) | low;
}

int8_t ByteBuffer::read_signed_byte() {
  return (int8_t)this->operator[](this->pointer++);
}

int16_t ByteBuffer::read_signed_short() {
  int16_t low = read_signed_byte();
  int16_t hi  = read_signed_byte();
  return (hi << 8) | low;
}

int32_t ByteBuffer::read_signed_int() {
  int32_t low = read_signed_short();
  int32_t hi  = read_signed_short();
  return (hi << 16) | low;
}

std::size_t ByteBuffer::seek(std::size_t offset, int whence) {
  switch(whence) {
    case SEEK_SET:
      pointer = offset;
      break;
    case SEEK_CUR:
      pointer += offset;
      break;
    case SEEK_END:
      pointer = size() + offset - 1;
      break;
  }
  return pointer;
}