/*
 * Package:   motion_planner
 * Filename:  lane_points.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef MOTION_PLANNER__LANE_POINTS_HPP_
#define MOTION_PLANNER__LANE_POINTS_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

namespace navigator
{
    namespace motion_planner
    {
        //represents a sampling of points for the left boundary, center line, and right boundary of a lane

        class LanePoints
        {
        public:
            const lanelet::LineString3d left;
            const lanelet::LineString3d center;
            const lanelet::LineString3d right;
            LanePoints(lanelet::LineString3d left, lanelet::LineString3d center, lanelet::LineString3d right)
                : left(left), center(center), right(right)
            {
            }
        };
    }
}

#endif
