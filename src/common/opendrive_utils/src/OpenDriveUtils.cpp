/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/OpenDriveUtils.hpp"
#include "Lanes.h"

using namespace navigator::opendrive;

/**
 * @brief loads an Opendrive map from a file name. 
 * 
 * Currently only calls on the OpenDriveMap constructor,
 * but may eventually include map validation and correction.
 * 
 * @param filename 
 * @return std::shared_ptr<odr::OpenDriveMap>
 */
OpenDriveMapPtr navigator::opendrive::load_map(const std::string & filename) {
    OpenDriveMapPtr map = std::make_shared<odr::OpenDriveMap>(filename);

    // Potential map validation and correction here.

    return map;
}


//TODO: remove this and put in utils package
//positive: is the lane_id positive? if so, we have to iterate in the opposite direction
odr::Line3D get_centerline_as_xy(const odr::Lane & lane, double s_start, double s_end, double ds, bool positive)
{
  odr::Line3D result;
    if (positive) {
        for (double s = s_end-ds; s >= s_start; s -= ds) {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s))/2;
	    odr::Vec3D pt = lane.get_surface_pt(s,t);
            result.push_back(pt);
        }
    } else {
        for (double s = s_start; s < s_end; s += ds) {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s))/2;
	    odr::Vec3D pt = lane.get_surface_pt(s,t);
            result.push_back(pt);
        }
    }
    return result;
}
