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

namespace navigator {
    namespace opendrive {

        using RoadId = std::string;
        using LaneSectionId = int;
        using LaneId = int;

        struct LaneIdentifier {
            RoadId road;
            LaneSectionId section;
            LaneId lane;
        };

        struct LaneLink {
            LaneIdentifier left;
            LaneIdentifier right;

            // Predecessor and successor guarenteed to be in correct driving direction.
            LaneIdentifier predecessor;
            LaneIdentifier successor;
        };

        class LaneGraph {
        public:
            LaneGraph(std::shared_ptr<odr::OpenDriveMap> map);

            // Retrieves a lane by identifier
            std::shared_ptr<odr::Lane> get_lane(LaneIdentifier lane_id);

        private:
            std::shared_ptr<odr::OpenDriveMap> map;
        };
    }
}