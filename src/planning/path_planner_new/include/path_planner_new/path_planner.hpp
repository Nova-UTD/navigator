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
            voltron_msgs::msg::CostedPath get_path(LaneletSequence lanelets);

            void set_map(lanelet::LaneletMapPtr map);

            // update planner state to new pose. Includes guessing current lanelet
            void set_pose(geometry_msgs::msg::Pose pose);

            void set_routing_costs(const std::vector<voltron_msgs::msg::RouteCost> &costs);
            double get_routing_cost(lanelet::Id lanelet);

            /**
             * @brief Get the forecast horizon in meters
             *
             * TODO: make this better or at least configurable
             *
             * @return distance in meters
             */
            double get_horizon() { return 100; }

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

            // Getter for current lanelet
            lanelet::Lanelet get_current_lanelet();

            rclcpp::Logger* logger = nullptr;
            void log(std::string msg){RCLCPP_WARN((*logger), msg);}
            void set_logger(rclcpp::Logger l){logger = &l; log("Logging working");}

        private:
            lanelet::LaneletMapPtr map;
            lanelet::routing::RoutingGraphUPtr routing_graph;
            lanelet::Lanelet current_lanelet;

            std::map<lanelet::Id, double> route_cost_map;
        };
    }
}

#endif