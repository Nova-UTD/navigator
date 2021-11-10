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

#include "path_planner/path_planner.hpp"

namespace navigator
{
    namespace path_planner
    {
        size_t get_closest_lanelet(const lanelet::ConstLanelets &lanelets, const TrajectoryPoint &point)
        {
            float64_t closest_distance = std::numeric_limits<float64_t>::max();
            size_t closest_index = 0;
            for (size_t i = 0; i < lanelets.size(); i++)
            {
                const auto &llt = lanelets.at(i);
                const auto &point2d = lanelet::Point2d(lanelet::InvalId, point.x, point.y).basicPoint2d();
                // TODO(mitsudome-r): change this implementation to remove dependency to boost
                const float64_t distance = lanelet::geometry::distanceToCenterline2d(llt, point2d);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_index = i;
                }
            }
            return closest_index;
        }
        float distance2d(const TrajectoryPoint &p1, const TrajectoryPoint &p2)
        {
            const auto diff = autoware::common::geometry::minus_2d(p1, p2);
            return autoware::common::geometry::norm_2d(diff);
        }
        lanelet::Point3d convertToLaneletPoint(
            const autoware_auto_msgs::msg::TrajectoryPoint &pt)
        {
            return lanelet::Point3d(lanelet::InvalId, pt.x, pt.y, 0.0);
        }
        ParameterizedSpline get_center_line_spline(const std::vector<lanelet::ConstPoint3d> &line_points)
        {
            std::vector<double> x_points(line_points.size());
            std::vector<double> y_points(line_points.size());
            for (const auto &p : line_points)
            {
                x_points.push_back(p.x());
                y_points.push_back(p.y());
            }
            return ParameterizedSpline(x_points, y_points);
        }
        std::vector<lanelet::ConstPoint3d> get_center_line_points(const HADMapRoute &route, const LaneletMapConstPtr &map, double resolution)
        {
            //a lot of this taken from the autoware implementation of lane_planner
            //add the lanes from the route
            lanelet::ConstLanelets lanelets;
            for (const auto &segment : route.segments)
            {
                const auto &primitive = segment.primitives.front();
                try
                {
                    const auto lane = map->laneletLayer.get(primitive.id);
                    lanelets.push_back(lane);
                }
                catch (const lanelet::NoSuchPrimitiveError &ex)
                {
                    // stop adding lanelets if lane cannot be found. e.g. goal is outside of queried submap
                    break;
                }
            }
            // return empty trajectory if there are no lanes
            if (lanelets.empty())
            {
                return TrajectoryPoints();
            }

            TrajectoryPoint trajectory_start_point;
            trajectory_start_point.x = static_cast<float>(had_map_route.start_point.position.x);
            trajectory_start_point.y = static_cast<float>(had_map_route.start_point.position.y);
            trajectory_start_point.heading = had_map_route.start_point.heading;

            TrajectoryPoint trajectory_goal_point;
            trajectory_goal_point.x = static_cast<float>(had_map_route.goal_point.position.x);
            trajectory_goal_point.y = static_cast<float>(had_map_route.goal_point.position.y);
            trajectory_goal_point.heading = had_map_route.goal_point.heading;

            const auto start_index = get_closest_lanelet(lanelets, trajectory_start_point);
            std::vector<lanelet::ConstPoint3d> line_points;
            // set position and velocity
            for (size_t i = start_index; i < lanelets.size(); i++)
            {
                const auto &lanelet = lanelets.at(i);
                const auto &centerline = autoware::common::had_map_utils::generateFineCenterline(
                    lanelet,
                    m_planner_config.trajectory_resolution);

                float64_t start_length = 0;
                if (i == start_index)
                {
                    const auto start_point = convertToLaneletPoint(trajectory_start_point);
                    start_length =
                        lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(start_point)).length;
                }

                float64_t end_length = std::numeric_limits<float32_t>::max();
                if (i == lanelets.size() - 1)
                {
                    const auto goal_point = convertToLaneletPoint(trajectory_goal_point);
                    end_length = lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(goal_point)).length;
                }

                float64_t accumulated_length = 0;
                // skip first point to avoid inserting overlaps
                for (size_t j = 1; j < centerline.size(); j++)
                {
                    const auto llt_prev_pt = centerline[j - 1];
                    const auto llt_pt = centerline[j];
                    accumulated_length += lanelet::geometry::distance2d(to2D(llt_prev_pt), to2D(llt_pt));
                    if (accumulated_length < start_length)
                    {
                        continue;
                    }
                    if (accumulated_length > end_length)
                    {
                        break;
                    }
                    line_points.push_back(llt_pt);
                }
            }
            line_points.push_back(trajectory_goal_point);
        }
    }
}