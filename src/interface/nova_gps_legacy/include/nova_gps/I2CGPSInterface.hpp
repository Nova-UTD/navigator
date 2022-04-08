#include "GPSInterface.hpp"
#include "I2CInterface.hpp"
#include "UBX.hpp"
#include <memory>
#include <string>
#include <vector>

namespace Nova {
  namespace GPS {
    class I2CGPSInterface : public GPSInterface {
      public:
        I2CGPSInterface(); // create without getting a handle
        I2CGPSInterface(const std::string & interface_name); // create and open

        virtual ~I2CGPSInterface();

        virtual void open(const std::string & interface_name) override;
        virtual void close() override;

        virtual bool gather_messages() override; // read and buffer some messages from the device
        virtual bool has_message() override; // returns true if get_message will succeed

        virtual std::unique_ptr<UBX::UBXMessage> get_message() override; // pops a message off the queue
        virtual void enqueue_outgoing_message(std::unique_ptr<UBX::UBXMessage> message) override;
        virtual bool send_messages() override;
      private:
        std::unique_ptr<Nova::I2C::I2CInterface> i2c_interface;
        std::vector<std::unique_ptr<UBX::UBXMessage>> nmea_message_buffer;
        std::vector<std::unique_ptr<UBX::UBXMessage>> send_queue;
    };
  }
}