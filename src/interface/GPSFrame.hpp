/*
 * Package:   gps
 * Filename:  include/gps/GPSFix.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "nav_msgs/msg/odometry.hpp"

namespace navigator {
namespace gps {

class GPSFix final {
public:
  GPSFix(std::string message);
  
  bool valid();
  nav_msgs::msg::Odometry to_message();

private:
  double latitude;
  double longtitude;
  double heading;
  double speed;
  double position_acc;
  double heading_acc;
  double speed_acc;
};

}
}
