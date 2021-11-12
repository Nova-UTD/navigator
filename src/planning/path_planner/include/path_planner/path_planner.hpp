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

#ifndef PATH_PLANNER__PATH_PLANNER_HPP_
#define PATH_PLANNER__PATH_PLANNER_HPP_

#include <path_planner/parameterized_spline.hpp>
#include <path_planner/lane_points.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <common/types.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <iostream>
#include <memory>
#include <vector>

using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Trajectory;


namespace navigator
{
    namespace path_planner
    {
        class PathPlanner
        {
        public:
            std::vector<TrajectoryPoint> generate_position_trajectory(const HADMapRoute &route, const lanelet::LaneletMapConstPtr &map);

        private:
            //probably need to find a way to bound start and end window
            ParameterizedSpline get_center_line_spline(const std::vector<lanelet::ConstPoint3d> &line_points);
            std::vector<lanelet::ConstPoint3d> get_center_line_points(const HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution);
        };
    }
}

#endif // PATH_PLANNER__PATH_PLANNER_HPP_
