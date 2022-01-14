/*
 * Package:   nova_gps
 * Filename:  GPSInterface.hpp
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

namespace Nova {
  namespace GPS {
    // :-)
    // see impl notes
    typedef std::string NMEAMessage;

    class GPSInterface {
      public:
        virtual ~GPSInterface() {}

        virtual void open(const std::string & interface_name) = 0;
        virtual void close() = 0;

        virtual bool gather_messages() = 0;
        virtual bool has_message() = 0;

        virtual std::unique_ptr<UBXMessage> get_message() = 0;

        virtual void write_config(Nova::ByteBuffer & config_message) = 0; // this needs a better interface
    };
  }
}