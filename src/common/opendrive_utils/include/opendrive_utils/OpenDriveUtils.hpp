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

#include "Lanes.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Road.h"
#include "RefLine.h"
#include "Junction.h"

#include <string>
#include <vector>
#include <cmath>
#include <unordered_set>

namespace navigator
{
  namespace opendrive
  {
    using namespace types;
    // positive: is the lane_id positive? if so, we have to iterate in the opposite direction
    odr::Line3D get_centerline_as_xy(const odr::Lane &lane, double s_start, double s_end, double ds, bool positive);
    double get_distance(std::shared_ptr<const odr::RefLine> line, double x, double y);
    LanePtr get_lane_from_xy(OpenDriveMapPtr map, double x, double y);
    LanePtr get_lane_from_xy(OpenDriveMapPtr map, double x, double y);
    LanePtr get_lane_from_xy_with_offset(OpenDriveMapPtr map, double x, double y, double offset);
    LanePtr get_lane_from_xy_with_route(OpenDriveMapPtr map, double x, double y, const std::set<std::string>& rs);
    std::shared_ptr<MapInfo> load_map(const std::string &filename);
    bool signal_applies(const Signal& signal, double s, const std::string& my_road_id, int my_lane_id);
    
    std::vector<std::shared_ptr<odr::Lane>> get_nearby_lanes(OpenDriveMapPtr map, double x, double y, double distance);

    void parse_signals(std::unordered_map<std::string, std::vector<Signal>>& dst, const std::string &filename);

    std::unordered_set<std::string> get_incoming_roads(OpenDriveMapPtr map, std::string junction_id);
  }
}
