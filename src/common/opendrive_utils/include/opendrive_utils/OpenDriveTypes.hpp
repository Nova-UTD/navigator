/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveTypes.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "OpenDriveMap.h"
#include <unordered_map>

namespace navigator
{
    namespace opendrive
    {
        namespace types
        {
            using OpenDriveMapPtr = std::shared_ptr<odr::OpenDriveMap>;
            using RoadPtr = std::shared_ptr<odr::Road>;
            using LaneSectionPtr = std::shared_ptr<odr::LaneSection>;
            using LanePtr = std::shared_ptr<odr::Lane>;

            using RoadId = std::string;
            using LaneSectionId = double;
            using LaneId = int;

            /**
             * @brief An identifier sufficient to uniquely identify a lane
             * regardless of context.
             *
             * @tparam RoadId
             * @tparam LaneSectionId: Unlike Road and Lane id, this is not
             *  present in the OpenDrive specification. LaneSections, within context
             *  of a Road, are uniquely identified by their s-offset: use s0 as id.
             * @tparam LaneId
             */
            struct LaneIdentifier
            {
                RoadId road;
                LaneSectionId section;
                LaneId lane;

                bool operator==(const LaneIdentifier &o) const
                {
                    return road == o.road && section == o.section && lane == o.lane;
                }

                bool operator<(const LaneIdentifier &o) const
                {
                    return road < o.road || (road == o.road && section < o.section) ||
                           (road == o.road && section == o.section && lane < o.lane);
                }
            };

            struct LaneLink
            {
                LaneIdentifier left;
                LaneIdentifier right;

                // Predecessor and successor guarenteed to be in correct driving direction.
                std::vector<LaneIdentifier> predecessors;
                std::vector<LaneIdentifier> successors;
            };

            //not parsed by libopendrive, so we add them ourselves
            struct Signal
            {
                std::string id;
                //"205" for yield
                //"206" for stop
                //should be replaced by enum
                std::string type;
                std::string name;
                //relative to road
                double s;
                double t;
                //does the signal change? ex: traffic lights = true, stop signs = false
                bool dynamic;
            };
            //stores the libopendrive map, as well as the other information they don't parse that we need (ex: singals)
            struct MapInfo
            {
                OpenDriveMapPtr map;
                //key is road id, value is list of all signals on that road
                std::unordered_map<std::string, std::vector<Signal>> signals;
            };
        }
    }
}