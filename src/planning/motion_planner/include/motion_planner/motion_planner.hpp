/*
 * Package:   motion_planner
 * Filename:  motion_planner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include <motion_planner/segmented_path.hpp>
#include <motion_planner/lane_points.hpp>
#include <motion_planner/path_point.hpp>
#include <motion_planner/ideal_point.hpp>
#include <motion_planner/car_pose.hpp>
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
    namespace motion_planner
    {
        //generates potential paths using line segements and gives them costs according to several factors (distance from center line/target, collisions, etc)
        class MotionPlanner
        {
        public:
            segmented_path get_center_line_segments(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points);
            std::vector<autoware_auto_msgs::msg::TrajectoryPoint> get_center_line_points(const HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution);
            std::shared_ptr<const std::vector<path_point>> get_trajectory(const std::vector<ideal_point> &ideal_path, const car_pose pose);
            double cost_path(const segmented_path &path, const std::vector<ideal_point> &ideal_path, const car_pose pose) const;
        private:
            //currently, these numbers are chosen as a guess. they will need to be determined later for safety.
            //(all units are in meters, seconds, radians if not mentioned)
            const size_t paths = 12; //number of paths to consider at a time
            const size_t points = 100; //number of points on path
            const double spacing = 0.1; //meters between points on path
            const double max_steering_angle = 0.5; //max angle the car can turn in radians
            const double max_steering_speed = 0.1; //max speed we can change the car's direction in radians/sec (ignoring speed)
            const double max_lateral_accel = 10; //max acceleration of the car on turns (used to prevent skidding/flipping)
            const double car_size_x = 1.5; //width of the car
            const double car_size_y = 3; //length of the car
        };
    }
}

#endif // MOTION_PLANNER__MOTION_PLANNER_HPP_
