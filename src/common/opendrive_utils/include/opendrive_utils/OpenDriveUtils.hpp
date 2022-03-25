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
#include "opendrive_utils/LaneGraph.hpp"

namespace navigator {
  namespace opendrive{
    using namespace types;
    OpenDriveMapPtr load_map(const std::string & filename);
  }
}
