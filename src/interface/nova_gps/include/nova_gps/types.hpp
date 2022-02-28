#pragma once
#include <cassert>
#include <memory>
namespace Nova {
template <typename T>
class Buffer
{
  public:
    Buffer<T>(std::size_t size);
    Buffer<T>(std::size_t size, std::unique_ptr<T[]> data);
    Buffer<T>(Buffer<T>&& o) noexcept : Buffer<T>(o.bufsize, std::move(o.data)) {};
    Buffer<T>(Buffer<T>&) = delete;
    T& operator[](unsigned int idx); 
    const T& operator[](unsigned int idx) const;
    std::size_t size() { return bufsize; };

  private:
    std::size_t bufsize;
    std::unique_ptr<T[]> data;
};

template class Buffer<uint8_t>;

class ByteBuffer : public Buffer<uint8_t>
{
  public:
    ByteBuffer(std::size_t size) : Buffer<uint8_t>(size) { pointer = 0; };
    ByteBuffer(std::size_t size, std::unique_ptr<uint8_t[]> data) : Buffer<uint8_t>(size, std::move(data)) { pointer = 0; };
    ByteBuffer(ByteBuffer&& o) noexcept : Buffer<uint8_t>(std::move(o)) { pointer = o.pointer;};
    ByteBuffer(ByteBuffer&) = delete;
    // all multibyte ops are LE
    uint8_t read_byte();
    uint16_t read_word();
    uint32_t read_dword();
    int8_t read_signed_byte();
    int16_t read_signed_short();
    int32_t read_signed_int();
    std::size_t get_pos() { return pointer; };
    std::size_t seek(std::size_t offset, int whence);
  private:
    std::size_t pointer;
};
}