/*
 * Package:   nova_gps
 * Filename:  GPSInterface.hpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#pragma once
#include "types.hpp"
#include "UBX.hpp"
#include <memory>
#include <string>
#include <vector>

namespace Nova {
  namespace GPS {
    class GPSInterface {
      public:
        virtual ~GPSInterface() {}

        virtual void open(const std::string & interface_name) = 0;
        virtual void close() = 0;

        virtual bool gather_messages() = 0;
        virtual bool has_message() = 0;

        virtual void enqueue_outgoing_message(std::unique_ptr<UBX::UBXMessage> message) = 0;
        virtual bool send_messages() = 0;
        
        virtual std::unique_ptr<UBX::UBXMessage> get_message() = 0;
    };
  }
}