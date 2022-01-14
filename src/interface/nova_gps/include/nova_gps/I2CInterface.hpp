/*
 * Package:   nova_gps
 * Filename:  I2CInterface.hpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#pragma once
#include "nova_gps/types.hpp"
#include <memory>
#include <string>
#include <vector>

// wraps a specific i2c device on a specific bus
namespace Nova {
    namespace I2C {
    class I2CInterface {
    public:
      I2CInterface(); // Initialize without opening
      I2CInterface(const uint8_t device_address, const std::string & interface_name); // Initialize and open
      I2CInterface(I2CInterface &&); // Move constructor
      ~I2CInterface();

      char read_byte(const uint8_t register_); // Read a byte at a specific register
      uint16_t read_word_pair(const uint8_t register_lo, const uint8_t register_hi); //read a word from a lo register and a hi register
      uint16_t read_word(const uint8_t register_); // read a word from the device
      std::unique_ptr<Nova::ByteBuffer> read_block(const uint8_t register_, const std::size_t count); // read a number of bytes from the device

      void write_byte(const uint8_t byte); // Write a byte to the device
      void write_block(Nova::ByteBuffer & block); // Write a block of data to the device
      virtual void open(const uint8_t device_address, const std::string & interface_name); // Open an interface
      virtual void close(); // Close the interface
      virtual bool is_open(); // Whether or not the bus is open
    private:
      virtual int device_read(uint8_t register_, uint8_t *value);
      virtual int device_write(uint8_t value);
      int i2c_device_handle;
      uint8_t device_address;
    };
  }
}
