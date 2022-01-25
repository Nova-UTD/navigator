#include "GPSInterface.hpp"
#include "I2CInterface.hpp"
#include <memory>
#include <string>
#include <vector>

namespace Nova {
  namespace GPS {
    class ConcreteGPSInterface : public GPSInterface {
      public:
        ConcreteGPSInterface(); // create without getting a handle
        ConcreteGPSInterface(const std::string & interface_name); // create and open

        virtual ~ConcreteGPSInterface();

        virtual void open(const std::string & interface_name) override;
        virtual void close() override;

        virtual bool gather_messages() override; // read and buffer some messages from the device
        virtual bool has_nmea_message() override; // returns true if get_nmea_message will succeed

        virtual std::unique_ptr<NMEAMessage> get_nmea_message() override; // pops a message off the queue

        virtual void write_config(const std::vector<uint8_t> & config) override; // pass a configuration string to the device

      private:
        std::unique_ptr<Nova::I2C::I2CInterface> i2c_interface;
        std::vector<NMEAMessage> nmea_message_buffer;
    };
  }
}