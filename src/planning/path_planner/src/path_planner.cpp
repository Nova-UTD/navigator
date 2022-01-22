/*
 * Package:   path_planner
 * Filename:  path_planner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "path_planner/path_planner.hpp"
#include "path_planner/map_utils.hpp"

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/LineString.h>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <had_map_utils/had_map_utils.hpp>
#include <common/types.hpp>
#include <geometry/common_2d.hpp>

using namespace autoware::common::types;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Trajectory;

using namespace navigator::path_planner;



size_t get_closest_lanelet(const lanelet::ConstLanelets &lanelets, const autoware_auto_msgs::msg::TrajectoryPoint &point)
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
lanelet::Point3d convertToLaneletPoint(
    const autoware_auto_msgs::msg::TrajectoryPoint &pt)
{
    return lanelet::Point3d(lanelet::InvalId, pt.x, pt.y, 0.0);
}
ParameterizedSpline MotionPlanner::get_center_line_spline(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points)
{
    std::vector<double> x_points(line_points.size());
    std::vector<double> y_points(line_points.size());
    for (const auto &p : line_points)
    {
        x_points.push_back(p.x);
        y_points.push_back(p.y);
    }
    return ParameterizedSpline(x_points, y_points);
}
autoware_auto_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(const lanelet::ConstPoint3d &pt)
{
    autoware_auto_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.x = static_cast<float32_t>(pt.x());
    trajectory_point.y = static_cast<float32_t>(pt.y());
    trajectory_point.longitudinal_velocity_mps = 0;
    return trajectory_point;
}
//resolution should be < 0.1 to get good approximation
std::vector<autoware_auto_msgs::msg::TrajectoryPoint> MotionPlanner::get_center_line_points(const HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution)
{
    using lanelet::utils::to2D;
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
        return std::vector<autoware_auto_msgs::msg::TrajectoryPoint>();
    }

    autoware_auto_msgs::msg::TrajectoryPoint trajectory_start_point;
    trajectory_start_point.x = static_cast<float>(route.start_point.position.x);
    trajectory_start_point.y = static_cast<float>(route.start_point.position.y);
    trajectory_start_point.heading = route.start_point.heading;

    autoware_auto_msgs::msg::TrajectoryPoint trajectory_goal_point;
    trajectory_goal_point.x = static_cast<float>(route.goal_point.position.x);
    trajectory_goal_point.y = static_cast<float>(route.goal_point.position.y);
    trajectory_goal_point.heading = route.goal_point.heading;

    const auto start_index = get_closest_lanelet(lanelets, trajectory_start_point);
    std::vector<autoware_auto_msgs::msg::TrajectoryPoint> line_points;
    // set position and velocity
    std::cout << "start index: " << start_index << std::endl;
    for (size_t i = start_index; i < lanelets.size(); i++)
    {
        const auto &lanelet = lanelets.at(i);
        const auto &centerline = map_utils::sample_center_line_and_boundaries(
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
        std::cout << "start length: " << start_length << std::endl;
        //arc length along lane for end point
        float64_t end_length = std::numeric_limits<float32_t>::max();
        if (i == lanelets.size() - 1)
        {
            const auto goal_point = convertToLaneletPoint(trajectory_goal_point);
            end_length = lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(goal_point)).length;
        }
        std::cout << "end length: " << end_length << std::endl;
        float64_t accumulated_length = 0;
        // skip first point on later iterations to avoid inserting overlaps
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
            line_points.push_back(convertToTrajectoryPoint(llt_pt));
        }
    }
    line_points.push_back(trajectory_goal_point);
    return line_points;
}