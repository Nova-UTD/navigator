/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// Contains every other opendrive_utils header, in addition to basic extra utility.

#pragma once

#include "OpenDriveMap.h"
#include "opendrive_utils/OpenDriveTypes.hpp"
#include "opendrive_utils/GeometricMap.hpp"
#include "opendrive_utils/LaneGraph.hpp"

#include "Lanes.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Road.h"
#include "RefLine.h"

#include <string>
#include <vector>
#include <cmath>

namespace navigator
{
  namespace opendrive
  {
    using namespace types;
    OpenDriveMapPtr load_map(const std::string &filename);
    LanePtr get_lane_from_xy(OpenDriveMapPtr map, double x, double y);
    odr::LaneSet get_nearby_lanes(OpenDriveMapPtr map, double x, double y, double distance);
  }
}