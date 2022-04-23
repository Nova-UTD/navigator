/*
 * Package:   gps
 * Filename:  include/gps/GPSFix.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <sstream>
#include "gps/GPSFrame.hpp"
using namespace navigator::gps;

GPSFrame::GPSFrame(std::string message) {
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

bool GPSFrame::valid() {
  return this->position_acc > 0;
}

nav_msgs::msg::Odometry GPSFrame::to_message() {
  nav_msgs::msg::Odometry msg;

  return msg;
}
