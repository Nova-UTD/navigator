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
        LanePoints sample_center_line_and_boundaries(
            const lanelet::ConstLanelet &lanelet_obj, const float64_t resolution)
        {
            // Get length of longer border
            const float64_t left_length =
                static_cast<float64_t>(lanelet::geometry::length(lanelet_obj.leftBound()));
            const float64_t right_length =
                static_cast<float64_t>(lanelet::geometry::length(lanelet_obj.rightBound()));
            const float64_t longer_distance = (left_length > right_length) ? left_length : right_length;
            const int32_t num_segments =
                std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

            // Resample points
            const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
            const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

            // Create centerline
            lanelet::LineString3d centerline(lanelet::utils::getId());
            lanelet::LineString3d leftBoundary(lanelet::utils::getId());
            lanelet::LineString3d rightBoundary(lanelet::utils::getId());
            for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++)
            {
                const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
                const lanelet::Point3d center_point(
                    lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
                    center_basic_point.z());
                centerline.push_back(center_point);
                leftBoundary.push_back(left_points.at(i));
                rightBoundary.push_back(right_points.at(i));
            }
            return LanePoints(liftBoundary, centerline, rightBoundary);
        }
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
                const auto &centerline = sample_center_line_and_boundaries(
                                             lanelet,
                                             resolution)
                                             .center;

                //arc length along lane for start point
                float64_t start_length = 0;
                if (i == start_index)
                {
                    const auto start_point = convertToLaneletPoint(trajectory_start_point);
                    start_length =
                        lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(start_point)).length;
                }
                //arc length along lane for end point
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
                    //may be able to make searching for the first length more effecient
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