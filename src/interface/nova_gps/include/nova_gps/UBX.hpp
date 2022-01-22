#pragma once
#include <memory>
#include <string>
#include <vector>
#include "nova_gps/types.hpp"
namespace Nova {
namespace UBX {

struct UBXMessage {
  uint8_t mclass;
  uint8_t id;
  uint16_t checksum;
  std::unique_ptr<ByteBuffer> data;
};


std::unique_ptr<std::vector<std::unique_ptr<UBXMessage>>> parse_ubx_messages
                (Nova::ByteBuffer & buf);

uint16_t calculate_checksum(Nova::UBX::UBXMessage & message);

std::unique_ptr<Nova::UBX::UBXMessage> set_message_rate(uint8_t message_class, uint8_t message_id, uint8_t rate);

enum GPSFix {
  NO_FIX,
  DEAD_RECKONING_ONLY,
  TWO_DIM_FIX,
  THREE_DIM_FIX,
  GPS_AND_DEAD_RECKONING,
  TIME_ONLY_FIX
};

// if you ever want to read this struct straight from memory,
// make sure to pack it
struct HNRPVT {
  uint32_t iTOW;   // ms, GPS Time-Of-Week of the current epoch
  uint16_t year;   // y
  uint8_t month;   // m, 1..12 (UTC)
  uint8_t day;     // d, 1..31 (UTC)
  uint8_t hour;    // h, 0..23 (UTC)
  uint8_t min;     // m, 0..59 (UTC)
  uint8_t sec;     // s, 0..59 (UTC)
  uint8_t valid;   // bitfield, 0 validDate 1 validTime 2 fullyResolved
  int32_t nano;    // ns, -1e9 .. 1e9 (UTC)
  uint8_t gpsFix;  // GPSFix
  uint8_t flags;   // bitfield, 0 GPSfixOK 1 DiffSoln 2 WKNSET 3 TOWSET 4 headVehValid
  uint8_t _1;      // unused
  uint8_t _2;      // unused
  int32_t lon;     // deg * 1e-7
  int32_t lat;     // deg * 1e-7
  int32_t height;  // mm, above elipsoid
  int32_t hMSL;    // mm, above mean sea level
  int32_t gSpeed;  // mm/s, 2-d ground speed
  int32_t speed;   // mm/s, 3-d speed
  int32_t headMot; // deg * 1e-5, heading of motion in 2d plane
  int32_t headVeh; // deg * 1e-5, heading of vehicle in 2d plane
  uint32_t hAcc;   // mm, horizontal accuracy
  uint32_t vAcc;   // mm, vertical accuracy
  uint32_t sAcc;   // mm/s, speed accuracy
  uint32_t headAcc;// deg * 1e-5, heading accuracy
  uint8_t __1;
  uint8_t __2;
  uint8_t __3;
  uint8_t __4;
};
struct CFGPRT_I2C {
  uint8_t portID;       // set to 0 for I2C
  uint8_t reserved1;
  uint16_t txReady;     // 32.10.25.5 Fig 1
  uint32_t mode;        // 32.10.25.5 Fig 2
  uint8_t reserved2[4]; 
  uint16_t inProtoMask; // 32.10.25.5 Fig 3
  uint16_t outProtoMask;// 32.10.25.5 Fig 4
  uint16_t flags;       // 32.10.25.5 Fig 5
  uint8_t reserved3[2];
};
}
}