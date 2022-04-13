/*
 * Package:   gps
 * Filename:  include/gps/GPSFix.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <sstream>
#include "gps/GPSFix.hpp"
using namespace navigator::gps;

GPSFix::GPSFix(std::string message) {
  std::stringstream stream(message);
  stream >> this->latitude;
  stream >> this->longtitude;
  stream >> this->heading;
  stream >> this->speed;
  stream >> this->position_acc;
  stream >> this->heading_acc;
  stream >> this->speed_acc;

  this->latitude /= 1000000; // degrees
  this->longtitude /= 1000000;
  this->heading /= 10000; // degrees
  this->speed /= 1000; // m/s
  this->position_acc /= 1000; // m
  this->heading_acc /= 10000; // degrees
  this->speed_acc /= 1000; // m/s
}

bool GPSFix::valid() {
  return this->position_acc > 0;
}

nav_msgs::msg::Odometry GPSFix::to_message() {
  nav_msgs::msg::Odometry msg;

  return msg;
}
