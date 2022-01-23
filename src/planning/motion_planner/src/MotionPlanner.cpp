/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "motion_planner/MotionPlanner.hpp"
#include "motion_planner/map_utils.hpp"

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

#include <math.h>

using namespace autoware::common::types;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Trajectory;

using namespace navigator::MotionPlanner;

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
SegmentedPath MotionPlanner::get_center_line_segments(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points)
{
    using namespace std;
    shared_ptr<vector<PathPoint>> points = make_shared<vector<PathPoint>>();
    for (const auto &p : line_points)
    {
        points->push_back(PathPoint(p.x,p.y));
    }
    return SegmentedPath(points);
}
autoware_auto_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(const lanelet::ConstPoint3d &pt)
{
    autoware_auto_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.x = static_cast<float32_t>(pt.x());
    trajectory_point.y = static_cast<float32_t>(pt.y());
    trajectory_point.longitudinal_velocity_mps = 0;
    return trajectory_point;
}
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

std::shared_ptr<const std::vector<PathPoint>> MotionPlanner::get_trajectory(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) {
    //generates a bunch of different paths for the car, assigns them costs, and picks the lowest cost path
    //if memory becomes an issue, could generate and cost the paths on demand, and recreate the lowest cost one.
    using namespace std;

    const double car_angle = atan2(pose.heading.imag, pose.heading.real);
    const double car_speed = sqrt(pose.longitudinal_v*pose.longitudinal_v+pose.lateral_v*pose.lateral_v);
    //no idea if this works like this. will have to test. cannot find documentation
    const double car_vx_norm = pose.heading.real;
    const double car_vy_norm = pose.heading.imag;
    const double max_steer_speed_time = max_steering_speed/car_speed; //convert from rad/m to rad/s

    shared_ptr<vector<PathPoint>> linear_points = make_shared<vector<PathPoint>>();
    for (size_t i = 0; i < points; i++) {
        linear_points->push_back(PathPoint(pose.x + car_vx_norm*i/spacing, pose.y+car_vy_norm*i/spacing));
    }
    
    auto base_path = SegmentedPath(linear_points);
    vector<SegmentedPath> candidates;

    //for now, keep start angle the same and vary the end angle and turn speed.
    //this should change to be more flexible later
    for (size_t i = 0; i < paths; i++) {
        double end_angle = -max_steering_angle+(i/4)*max_steering_angle;
        double turn_speed = -max_steering_speed+(i%4)*max_steering_speed;
        auto branch_points = base_path.create_branch(car_angle,end_angle,turn_speed,points);
        candidates.push_back(SegmentedPath(branch_points));
    }

    //find min cost path
    double min_cost = -1;
    size_t min_index = -1;
    for (size_t i = 0; i < candidates.size(); i++) {
        double cost = cost_path(candidates[i], ideal_path, pose);
        if (cost < min_cost || cost == -1) {
            min_cost = cost;
            min_index = i;
        }
    }

    return candidates[min_index].points;
}

double MotionPlanner::cost_path(const SegmentedPath &path, const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) const {
    //temporary cost function, very basic attempt at following the ideal path.
    double dist = 0;
    for (const auto ideal : ideal_path.points) {
        dist += path.distance(PathPoint(ideal.x,ideal.z));
    }
    return dist;
    /*path.sum([&](PathPoint p) {
        can do comfortability cost function like this
    });*/
}