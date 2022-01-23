/*
 * Package:   path_planner
 * Filename:  path_planner.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef PATH_PLANNER__PATH_PLANNER_HPP_
#define PATH_PLANNER__PATH_PLANNER_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <geometry_msgs/msg/pose.hpp>
#include <voltron_msgs/msg/route_costs.hpp>
#include <voltron_msgs/msg/costed_path.hpp>

#include <rclcpp/rclcpp.hpp>


namespace navigator
{
    namespace path_planner
    {

        using LaneletSequence = std::vector<lanelet::Id>;

        class PathPlanner
        {
        public:
            PathPlanner();

            // The main external-facing method to generate all paths and write to results
            void get_paths(
                const geometry_msgs::msg::Pose &current_pose,
                const std::vector<voltron_msgs::msg::RouteCost> &route_costs,
                std::vector<voltron_msgs::msg::CostedPath> &results);

            // From a lanelet sequence get a single space-path
            voltron_msgs::msg::CostedPath get_path(const LaneletSequence &lanelets, const geometry_msgs::msg::Pose &current_pose);

            // Generate a sequence of lanelets where the last one is guarenteed to contain a point
            // "distance" meters away from the starting point
            void get_lanelet_sequences(double distance,
                                       const geometry_msgs::msg::Point starting_point,
                                       lanelet::ConstLanelet starting_lanelet,
                                       std::vector<LaneletSequence> &results);
            // Recursive method for generating the sequence described above
            void _get_lanelet_sequences(double remaining_distance,
                                        LaneletSequence sequence, lanelet::ConstLanelet current,
                                        std::vector<LaneletSequence> &results);


            // Setters and methods to update planner state
            
            // sets map and create routing graph
            void set_map(lanelet::LaneletMapPtr map);

            // update planner state to new pose. Includes guessing current lanelet
            void set_pose(geometry_msgs::msg::Pose pose);

            void set_routing_costs(const std::vector<voltron_msgs::msg::RouteCost> &costs);

            /**
             * @brief Get the forecast horizon in meters
             *
             * TODO: make this better or at least configurable
             *
             * @return distance in meters
             */
            double get_horizon() { return 100; }

            // Getter for current lanelet
            lanelet::Lanelet get_current_lanelet();
                        
            double get_routing_cost(lanelet::Id lanelet);

            // Utilities

            // Checks adjacencey (immediate left/immediate right) between the two lanelets
            bool is_adjacent(const lanelet::ConstLanelet l1, const lanelet::ConstLanelet l2);

            // Converts message point to lanelet point
            static lanelet::BasicPoint2d ros_point_to_lanelet_point(geometry_msgs::msg::Point point);
            static geometry_msgs::msg::Point lanelet_point_to_ros_point(lanelet::BasicPoint2d  point);


            // Splits a linestring at a point. The point does not need to be on the linestring.
            static void split_linestring(const lanelet::ConstLineString2d original, lanelet::BasicPoint2d split_point,
                                         lanelet::BasicLineString2d &first, lanelet::BasicLineString2d &second);

        private:
            lanelet::LaneletMapPtr map;
            lanelet::routing::RoutingGraphUPtr routing_graph;
            lanelet::Lanelet current_lanelet;

            std::map<lanelet::Id, double> route_cost_map;
        };
    }
}

#endif