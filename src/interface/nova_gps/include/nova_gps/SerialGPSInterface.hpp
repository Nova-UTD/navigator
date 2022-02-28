#include "GPSInterface.hpp"
#include "I2CInterface.hpp"
#include "UBX.hpp"
#include <memory>
#include <string>
#include <vector>

namespace Nova {
  namespace GPS {
    class SerialGPSInterface : public GPSInterface {
      public:
        SerialGPSInterface(); // create without getting a handle
        SerialGPSInterface(const std::string & interface_name); // create and open

        virtual ~SerialGPSInterface();

        virtual void open(const std::string & interface_name) override;
        virtual void close() override;

        virtual bool gather_messages() override; // read and buffer some messages from the device
        virtual bool has_message() override; // returns true if get_message will succeed

        virtual std::unique_ptr<UBX::UBXMessage> get_message() override; // pops a message off the queue
        virtual void enqueue_outgoing_message(std::unique_ptr<UBX::UBXMessage> message) override;
        virtual bool send_messages() override;
      private:
        int serial_device_handle;
        std::vector<std::unique_ptr<UBX::UBXMessage>> nmea_message_buffer;
        std::vector<std::unique_ptr<UBX::UBXMessage>> send_queue;
    };
  }
}