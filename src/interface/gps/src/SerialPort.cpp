/*
 * Package:   gps
 * Filename:  src/SerialPort.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "gps/SerialPort.hpp"
#include <cstring>
#include <stdexcept>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using navigator::gps::SerialPort;

SerialPort::SerialPort(const std::string & device_name) {
  this->descriptor = open(device_name.c_str(), O_RDWR);
  if(this->descriptor < 0) throw std::runtime_error("Error from open");

  struct termios tty;
  if(tcgetattr(this->descriptor, &tty) != 0) throw std::runtime_error("Error from tcgetattr");

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 1;
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if(tcsetattr(this->descriptor, TCSANOW, &tty) != 0) throw std::runtime_error("Error from tcsetattr");
}

std::string SerialPort::get_line() {
  std::string buffer;
  while(true) {
    char input;
    if(read(this->descriptor, &input, 1)) {
      if(input == '\n') {
	return buffer;
      } else {
	buffer += input;
      }
    }
  }
}
