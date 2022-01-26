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

std::shared_ptr<std::vector<SegmentedPath>> MotionPlanner::get_trajectory(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) {
    //generates a bunch of different paths for the car, assigns them costs, and picks the lowest cost path
    //if memory becomes an issue, could generate and cost the paths on demand, and recreate the lowest cost one.
    using namespace std;

    const double car_angle = pose.heading;
    const double car_speed = sqrt(pose.xv*pose.xv+pose.yv*pose.yv);
    //no idea if this works like this. will have to test. cannot find documentation
    const double car_vx_norm = cos(car_angle);
    const double car_vy_norm = sin(car_angle);
    double max_steer_speed_time = 1;
    if (car_speed != 0) {
        max_steer_speed_time = max_steering_speed/car_speed; //convert from rad/m to rad/s
    }

    shared_ptr<vector<PathPoint>> linear_points = make_shared<vector<PathPoint>>();
    for (size_t i = 0; i < points; i++) {
        double index = static_cast<double>(i);
        linear_points->push_back(PathPoint(pose.x + car_vx_norm*index*spacing, pose.y+car_vy_norm*index*spacing));
    }
    
    auto candidates = std::make_shared<vector<SegmentedPath>>();
    auto base = SegmentedPath(linear_points);
    auto bounds = get_path_bounds(ideal_path, pose);
    base.cost = cost_path(base, ideal_path, pose, bounds.first, bounds.second);
    candidates->push_back(base);

    for (size_t s = 0; s < steering_speeds; s++) {
        double turn_speed = -max_steering_speed+max_steering_speed*2*(static_cast<double>(s)/static_cast<double>(steering_speeds));
        for (size_t a = 0; a < steering_angles; a++) {
            double angle_change = -max_steering_angle+max_steering_angle*2*(static_cast<double>(s)/static_cast<double>(steering_angles));
            auto branch_points = candidates->at(0).create_branch(car_angle,angle_change,turn_speed,points);
            auto path = SegmentedPath(branch_points);
            auto bounds = get_path_bounds(ideal_path, pose);
            path.cost = cost_path(path, ideal_path, pose, bounds.first, bounds.second);
            candidates->push_back(path);
        }
    }
    return candidates;
}

double MotionPlanner::cost_path(const SegmentedPath &path, const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose, size_t start, size_t end) const {
    double dist = INFINITY;
    if (start <= end) {
        dist = 0;
        for (size_t i = start; i < end; i++) {
            dist += path.distance(PathPoint(ideal_path.points[i].x,ideal_path.points[i].y));
        }
    } else {
        dist = 0;
        for (size_t i = start; i > end; i--) {
            dist += path.distance(PathPoint(ideal_path.points[i].x,ideal_path.points[i].y));
        }
    }
    return dist;
}

std::pair<size_t,size_t> MotionPlanner::get_path_bounds(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) const {
    //find closest point to car on ideal path
    size_t closest = 0;
    bool found = false;
    double min_dist = INFINITY;
    for (size_t i = 0; i < ideal_path.points.size(); i++) {
        const auto ideal = ideal_path.points[i];
        double d = (pose.x-ideal.x)*(pose.x-ideal.x)+(pose.y-ideal.y)*(pose.x-ideal.y);
        if (d < min_dist) {
            closest = i;
            min_dist = d;
            found = true;
        }
    }

    if (!found) {
        return std::make_pair<size_t,size_t>(0,0); //no closest point found??
    }

    //find direction of car relative to ideal path (should we iterate forward or backward for distance horizon?)
    int direction = 1;
    if (closest == ideal_path.points.size()-1) direction = -1;
    if (ideal_path.points.size() > 1) {
        //if dot product of the direction the path is going and the car heading is non-negative,
        //we iterate forward (the car is going in the same direction as the path)
        PathPoint dir_vec = PathPoint(ideal_path.points[closest+1].x-ideal_path.points[closest].x,ideal_path.points[closest+1].y-ideal_path.points[closest].y);
        PathPoint heading_vec = PathPoint(cos(pose.heading), sin(pose.heading));
        double dot = dir_vec.x*heading_vec.x+dir_vec.y*heading_vec.y;
        direction = (dot >= 0) ? 1 : -1;
    }

    //find horizon. iterate along the ideal path until the arclength is more than the horizon
    // or we hit either end of the path lol
    double acc_dist_sqr = 0;
    size_t end = closest+direction;
    auto prev = ideal_path.points[closest];
    while (acc_dist_sqr < horizon*horizon && end < ideal_path.points.size()) {
        auto curr = ideal_path.points[end];
        acc_dist_sqr = (curr.x-prev.x)*(curr.x-prev.x)+(curr.y-prev.y)*(curr.y-prev.y);
        if (end == 0 && direction == -1) break; //avoid underflow (at bound anyway)
        end += direction;
    }

    return std::make_pair(closest,end);
}