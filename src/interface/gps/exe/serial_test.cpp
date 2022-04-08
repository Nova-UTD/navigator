#include <iostream>
#include "gps/SerialPort.hpp"

int main(int argc, char ** argv) {
  if(argc < 2) {
    std::cout << "Please provide a device name" << std::endl;
    return 1;
  }

  std::string device(argv[1]);
  navigator::gps::SerialPort serial(device);

  while(true) {
    std::cout << serial.get_line() << std::endl;
  }
}
