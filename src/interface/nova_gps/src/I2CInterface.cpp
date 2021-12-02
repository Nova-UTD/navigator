/*
 * Package:   nova_gps
 * Filename:  I2Cinterface.cpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "nova_gps/I2CInterface.hpp"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define FD_INVALID -1

using namespace Nova::I2C;

// resolve name collision
inline int open_syscall(const char * file, int oflag) {
    return open(file, oflag);
}

inline int close_syscall(int fd) {
    return close(fd);
}


int i2c_action(int device, char rw, uint8_t command, int size, union i2c_smbus_data *data) {
    struct i2c_smbus_ioctl_data arguments;
    arguments.command = command;
    arguments.data = data;
    arguments.read_write = rw;
    arguments.size = size;
    int status = ioctl(device, I2C_SMBUS, &arguments);
    if(status == -1) {
        status = -errno;
    }
    return status;
}

int device_write(int device, uint8_t value) {
	return i2c_action(device, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE_DATA, nullptr);
}

int device_read(int device, uint8_t register_, uint8_t *value) {
    union i2c_smbus_data data;
	int err = i2c_action(device, I2C_SMBUS_READ, register_, I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;
    *value = data.byte;
	return 1;
}

I2CInterface::I2CInterface() {
    this->i2c_device_handle = FD_INVALID;
}

I2CInterface::I2CInterface(const uint8_t device_address, const std::string & interface_name) {
    this->i2c_device_handle = FD_INVALID;
    this->open(device_address, interface_name);
}

I2CInterface::I2CInterface(I2CInterface && src) {
    this->i2c_device_handle = src.i2c_device_handle;
    src.i2c_device_handle = FD_INVALID;
}

void I2CInterface::open(uint8_t device_address, const std::string &interface_name) {
    // c land
    this->i2c_device_handle = open_syscall(interface_name.c_str(), O_RDWR);
    if(i2c_device_handle < 0) {
      throw std::runtime_error("Error while opening I2C device: errno is " +
			       std::to_string(errno));
    }
    if(ioctl(this->i2c_device_handle, I2C_SLAVE, device_address) < 0) {
      throw std::runtime_error("Error while setting I2C address: errno is " +
			       std::to_string(errno));
    }
    unsigned long flags;
    if(ioctl(this->i2c_device_handle, I2C_FUNCS, &flags) < 0) {
        throw std::runtime_error("Error while detecting I2C capabilities: errno is " +
                    std::to_string(errno));
    }
    //this->block_transfers = flags & I2C_FUNC_SMBUS_BLOCK_DATA;
    // end c land
}

void I2CInterface::close() {
    if(this->is_open()) {
        close_syscall(this->i2c_device_handle);
        this->i2c_device_handle = FD_INVALID;
    }
}

inline int internal_write_n_bytes(const int handle, const uint8_t * buf, int number_of_bytes, bool fail_if_less = true) {
    int bytes_written = 0;
    for(int i = 0; i < number_of_bytes; i++) {
        bytes_written += device_write(handle, buf[i]);
    }
    if(fail_if_less && bytes_written != number_of_bytes) {
        throw std::runtime_error("Error while writing to I2C device: errno is "+
                    std::to_string(errno));
    }
    return bytes_written;
}

inline int internal_read_block(const int handle, std::vector<char> store_to, int number_of_bytes, const uint8_t register_, bool fail_if_less = true) {
    int bytes_read = 0;
    for(int i = 0; i < number_of_bytes; i++) {
        uint8_t last_read;
        bytes_read += device_read(handle, register_, &last_read);
        //todo: buffers really aren't vectors... really we just want a nice wrapper around a fixed size block of memory
        store_to.push_back(last_read);
    }
    if(fail_if_less && bytes_read != number_of_bytes) {
        throw std::runtime_error("Error while reading from I2C device: errno is "+
                        std::to_string(errno));
    }
    return bytes_read;
}

inline int internal_read_byte(const int handle, uint8_t * value, const uint8_t register_, bool fail_if_less = true) {
    if(fail_if_less && device_read(handle, register_, value) != 1) {
        throw std::runtime_error("Error while reading from I2C device: errno is "+
                        std::to_string(errno));
    }
    return 1;
}


void I2CInterface::write_byte(const uint8_t byte) {
    const uint8_t buf[1] = {byte};
    internal_write_n_bytes(this->i2c_device_handle, buf, 2);
}

char I2CInterface::read_byte(const uint8_t register_) {
    char buf[1] = {0};
    internal_read_byte(this->i2c_device_handle, (uint8_t*)buf, register_);
    return buf[0];
}
uint16_t I2CInterface::read_word_pair(const uint8_t register_lo, const uint8_t register_hi) {
    uint8_t buf[2] = {register_lo, register_hi};
    internal_read_byte(this->i2c_device_handle, buf, register_lo);
    internal_read_byte(this->i2c_device_handle, buf+1, register_hi);
    return (uint16_t)(buf[1]) | (uint16_t)(buf[0] << 8);
}

uint16_t I2CInterface::read_word(const uint8_t register_) {
    uint8_t buf[2] = {0, 0};
    internal_read_byte(this->i2c_device_handle, buf, register_);
    return (uint16_t)(buf[1]) | (uint16_t)(buf[0] << 8);
}

std::unique_ptr<std::vector<char>> I2CInterface::read_block(const uint8_t register_, const int count) {
    std::unique_ptr<std::vector<char>> buffer = std::unique_ptr<std::vector<char>>();
    buffer->reserve(count);
    internal_read_block(this->i2c_device_handle, *buffer, count, register_);
    return buffer;
}

void I2CInterface::write_block(const std::vector<uint8_t> & block) {
    internal_write_n_bytes(this->i2c_device_handle, &(block)[0], block.size());
}

bool I2CInterface::is_open() {
    return this->i2c_device_handle > 0;
}

I2CInterface::~I2CInterface() {
    close();
}
