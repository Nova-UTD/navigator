/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/OpenDriveUtils.hpp"

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