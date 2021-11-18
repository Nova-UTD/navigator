/*
 * Package:   nova_gps
 * Filename:  GPSInterface.hpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#pragma once

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
        virtual bool has_nmea_message() = 0;

        virtual std::unique_ptr<NMEAMessage> get_nmea_message() = 0;

        virtual void write_config(const std::vector<uint8_t> & config_message) = 0; // this needs a better interface
    };
  }
}