/*
 * Package:   opendrive_utils
 * Filename:  LaneGraph.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/OpenDriveUtils.hpp"
#include "opendrive_utils/LaneGraph.hpp"

using namespace navigator::opendrive;
using namespace navigator::opendrive::types;

LaneGraph::LaneGraph(OpenDriveMapPtr map)
{
    this->map = map;
    for (RoadPtr r : map->get_roads())
    {
        RoadId r_id = r->id;
        for (LaneSectionPtr ls : r->get_lanesections())
        {
            LaneSectionId ls_id = ls->s0;
            for (LanePtr l : ls->get_lanes())
            {
                LaneId l_id = l->id;
                LaneIdentifier lane_id = {r_id, l_id};
                lanes[lane_id] = l;
            }
        }
    }
}

LanePtr LaneGraph::get_lane(LaneIdentifier lane_id)
{
    return lanes[lane_id];
}