/*
 * Package:   opendrive_utils
 * Filename:  LaneGraph.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "OpenDriveMap.h"
#include "opendrive_utils/OpenDriveTypes.hpp"

namespace navigator {
    namespace opendrive {

        using namespace types;

        class LaneGraph {
        public:
            LaneGraph(OpenDriveMapPtr map);

            // Retrieves a lane by identifier
            LanePtr get_lane(LaneIdentifier lane_id);
            
            OpenDriveMapPtr get_map(){
                return map;
            }

        private:
            OpenDriveMapPtr map;
            std::map<LaneIdentifier, LanePtr> lanes;
        };
    }
}