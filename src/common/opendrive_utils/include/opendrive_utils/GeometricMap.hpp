/*
 * Package:   opendrive_utils
 * Filename:  GeometricMap.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once
#include "OpenDriveMap.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace navigator
{
  namespace opendrive
  {
    class GeometricMap final
    {
    public:
      GeometricMap(const odr::OpenDriveMap &map);

    private:
      unsigned int number_of_roads;
    };
  }
}
