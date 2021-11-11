// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lane_planner class.

#ifndef PATH_PLANNER__LANE_POINTS_HPP_
#define PATH_PLANNER__LANE_POINTS_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

namespace navigator
{
    namespace path_planner
    {
        //represents a sampling of points for the left boundary, center line, and right boundary of a lane

        class LanePoints
        {
        public:
            const lanelet::LineString3d left;
            const lanelet::LineString3d center;
            const lanelet::LineString3d right;
            LanePoints(lanelet::LineString3d left, lanelet::LineString3d center, lanelet::LineString3d right)
                : left(left), cneter(center), right(right)
            {
            }
        };
    }
}

#endif
