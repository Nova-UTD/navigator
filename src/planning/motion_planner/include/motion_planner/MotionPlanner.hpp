/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef MotionPlanner__MotionPlanner_HPP_
#define MotionPlanner__MotionPlanner_HPP_

#include <motion_planner/SegmentedPath.hpp>
#include <motion_planner/LanePoints.hpp>
#include <motion_planner/PathPoint.hpp>
#include <motion_planner/IdealPoint.hpp>
#include <motion_planner/CarPose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <voltron_msgs/msg/costed_path.hpp>
#include <common/types.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <iostream>
#include <memory>
#include <vector>

namespace navigator
{
    namespace MotionPlanner
    {
        //generates potential paths using line segements and gives them costs according to several factors (distance from center line/target, collisions, etc)
        class MotionPlanner
        {
        public:
            SegmentedPath get_center_line_segments(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points);
            std::vector<autoware_auto_msgs::msg::TrajectoryPoint> get_center_line_points(const autoware_auto_msgs::msg::HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution);
            std::shared_ptr<const std::vector<PathPoint>> get_trajectory(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose);
            double cost_path(const SegmentedPath &path, const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) const;
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

#endif // MotionPlanner__MotionPlanner_HPP_
