/*
 * Package:   MotionPlanner
 * Filename:  LanePoints.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef MotionPlanner__LanePoints_HPP_
#define MotionPlanner__LanePoints_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

namespace navigator
{
    namespace MotionPlanner
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
