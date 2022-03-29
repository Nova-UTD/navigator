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
#include "Lanes.h"
#include "opendrive_utils/OpenDriveTypes.hpp"
#include "opendrive_utils/LaneGraph.hpp"

namespace navigator {
  namespace opendrive{
    using namespace types;
    OpenDriveMapPtr load_map(const std::string & filename);
    //positive: is the lane_id positive? if so, we have to iterate in the opposite direction
    odr::Line3D get_centerline_as_xy(const odr::Lane& lane, double s_start, double s_end, double ds, bool positive);
    double get_distance(std::shared_ptr<const odr::RefLine> line, double x, double y);
    std::shared_ptr<const odr::Road> get_road_from_xy(const odr::OpenDriveMap* map, double x, double y);
    std::shared_ptr<odr::Lane> get_lane_from_xy(const odr::OpenDriveMap* map, double x, double y);
    std::shared_ptr<odr::Lane> get_lane_from_xy_with_route(const odr::OpenDriveMap* map, double x, double y, const std::set<std::string>& rs);
  }
}


