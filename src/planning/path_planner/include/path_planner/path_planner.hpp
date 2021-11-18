/*
 * Package:   path_planner
 * Filename:  path_planner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

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
            ParameterizedSpline get_center_line_spline(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points);
            std::vector<autoware_auto_msgs::msg::TrajectoryPoint> get_center_line_points(const HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution);

        private:
            //probably need to find a way to bound start and end window
            
        };
    }
}

#endif // PATH_PLANNER__PATH_PLANNER_HPP_
