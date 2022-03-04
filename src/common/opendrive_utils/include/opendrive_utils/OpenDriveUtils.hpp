/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "OpenDriveMap.h"
#include "opendrive_utils/GeometricMap.hpp"
#include "opendrive_utils/LaneGraph.hpp"

namespace navigator {
  namespace opendrive{
    using OpenDriveMapPtr = std::shared_ptr<odr::OpenDriveMap>;
    
    OpenDriveMapPtr load_map(const std::string & filename);

  }
}