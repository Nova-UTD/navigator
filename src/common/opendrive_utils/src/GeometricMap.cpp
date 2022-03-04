/*
 * Package:   opendrive_utils
 * Filename:  GeometricMap.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/GeometricMap.hpp"
using navigator::GeometricMap;
using odr::OpenDriveMap;

GeometricMap::GeometricMap(const OpenDriveMap & map) {
  this->number_of_roads = map.get_roads().size();
}

unsigned int GeometricMap::get_n_roads() {
  return this->number_of_roads;
}
