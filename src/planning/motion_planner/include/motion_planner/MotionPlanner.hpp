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
#include <motion_planner/Collision.hpp>
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
#include <utility>

namespace navigator
{
    namespace MotionPlanner
    {
        //generates potential paths using line segements and gives them costs according to several factors (distance from center line/target, collisions, etc)
        class MotionPlanner
        {
        public:
            //currently unused. this converts a vector of trajectory points to a segmented path.
            //will decide whether or not this needs to be deleted when I implement cost function on lane boundaries
            SegmentedPath get_center_line_segments(const std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &line_points);
            //currently unused. converts the center line of a route into a vector of trajectory points.
            //will decide whether or not this needs to be deleted when I implement cost function on lane boundaries
            std::vector<autoware_auto_msgs::msg::TrajectoryPoint> get_center_line_points(const autoware_auto_msgs::msg::HADMapRoute &route, const lanelet::LaneletMapConstPtr &map, double resolution);
            //generates and costs a vector of candidate immedate trajectories the car could follow
            std::shared_ptr<std::vector<SegmentedPath>> get_trajectory(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose, const std::vector<CarPose>& colliders);
            //gets all potential collision events along the specified path given the objects to collide with
            std::vector<Collision> get_collisions(const SegmentedPath& path, const std::vector<CarPose>& objects) const;
            //assigns a velocity to each point in the path, respecting vehicle parameters, avoiding collisions, and limiting the max speed
            //COLLISIONS SHOULD BE SORTED BY CLOSEST FIRST
            //returns the sum of 1/(potential collision time) for all collisions that cross the path
            double assign_velocity(const voltron_msgs::msg::CostedPath ideal_path, SegmentedPath& assignee, const CarPose& my_pose, const std::vector<double>& bp_speed_limit, const std::vector<Collision>& collisions) const;
            //
            double cost_path(const SegmentedPath &path, const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose, size_t start, size_t end) const;
            //currently, these numbers are chosen as a guess. they will need to be determined later for safety.
            //(all units are in meters, seconds, radians if not mentioned)
            const size_t points = 100; //number of points on path
            const size_t steering_speeds = 16; //number of different steering speeds to consider
            const size_t steering_angles = 2; //number of different steering angles to consider
            const double spacing = 0.25; //meters between points on path
            const double max_steering_angle = 1; //max angle the car can turn in radians
            const double max_steering_speed = 0.1; //max speed we can change the car's direction in radians/sec (ignoring speed)
            const double max_lateral_accel = 10000; //max acceleration of the car on turns (used to prevent skidding/flipping)
            const double max_accel = 1;
            const double max_brake_accel = 5;
            const double car_size_x = 1.5; //width of the car
            const double car_size_y = 3; //length of the car
            const double horizon = points*spacing; //max distance to consider anything for cost 
            const double following_time = 2; //safe following time (seconds). This is also used to pad the time for all collisions.
            const double following_distance = 1; //safe following distance (m). This is also used to pad the distance for all collisions.
        private:
            //gets iteration bounds on the chosen input path based on car position and the horizon size
            std::pair<size_t,size_t> get_path_bounds(const voltron_msgs::msg::CostedPath ideal_path, const CarPose pose) const;
            void put_speed(SegmentedPath& assignee, const std::vector<double>& speed) const;
        };
    }
}

#endif // MotionPlanner__MotionPlanner_HPP_
