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
        virtual bool has_message() override; // returns true if get_message will succeed

        virtual std::unique_ptr<UBXMessage> get_message() override; // pops a message off the queue

        virtual void write_config(Nova::ByteBuffer & config) override; // pass a configuration string to the device

      private:
        std::unique_ptr<Nova::I2C::I2CInterface> i2c_interface;
        std::vector<std::unique_ptr<UBXMessage>> nmea_message_buffer;
    };
  }
}