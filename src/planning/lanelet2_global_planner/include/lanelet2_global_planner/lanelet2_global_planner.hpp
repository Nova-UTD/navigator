/*
 * Package:   lanelet2_global_planner
 * Filename:  lanelet2_global_planner.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// This file lays out the logic half of the route planner, implemented
// in lanelet2_global_planner.cpp. It is intended for use when the destination
// is change infrequently but the current position/origin changes often.
// It provides routing costs paired with lanelet IDs. Routing costs are 
// relative to the routing cost of the current position:
//  - a cost of 0 means moving to that lanelet from current does not
//    move closer to or farther from the goal
//  - a positive cost means that moving to that lanelet from current
//    moves further from the goal
//  - a negative cost means moving to that lanelet from current moves
//    closer to the goal


// The following aspects of this file and the implementation need improvement:
//  - Configurability: Pull values like horizon from a param file
//  - Cost flexibility: add in the custom cost function. WARNING: THIS MAY
//    CAUSE TESTS TO FAIL: SEE THE TODO COMMENT IN TEST FILE


#ifndef LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
#define LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_

// lanelet2
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// autoware
#include <lanelet2_global_planner/visibility_control.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <common/types.hpp>
// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <regex>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace autoware
{
  namespace planning
  {
    namespace lanelet2_global_planner
    {

      using autoware_auto_msgs::msg::TrajectoryPoint;
      
      using LaneRouteCost = std::pair<lanelet::Id, float64_t>;
      using LaneRouteCosts = std::vector<LaneRouteCost>;

      class LANELET2_GLOBAL_PLANNER_PUBLIC Lanelet2GlobalPlanner
      {
      public:
        Lanelet2GlobalPlanner();

         /**
         * Fetches lanelets near current location and labels them with routing cost.
         * 
         * If the location is not inside a lanelet the closest is used.
         * 
         * Writes <Id, cost> pairs to a new vector passed back to provided address
         */
        void fetch_routing_costs(TrajectoryPoint &location, LaneRouteCosts &costs) const;

        /**
         * Changes goal destination of the routing planner.
         * 
         * Updates internal costs for the whole map: should be called infrequently. 
         */
        void set_destination(TrajectoryPoint &goal);

        /**
         * Finds a number lanelets that contain the location.
         *
         * If any lanelets are found that contain the point, they are added
         * and a true value is returned.
         *
         * If no lanelets are found that contain the point, the next closest
         * is found but a false value is returned.
         *
         * @param location
         * @param depth - find up to this number of lanelets
         * @param result_wb - lanelets found are appended here
         * @return true if lanelets were found containing the point
         */
        bool find_lanelets(TrajectoryPoint location, uint depth, std::vector<lanelet::Lanelet> &result_wb) const;

        /**
         * Gets the routing cost associated with a lanelet IF the destination
         * can be reached from that lanelet.
         * 
         * @param id 
         * @param cost_writeback - return by reference cost value. Only modified if cost exists.
         * @return true if the destination can be reached from the lanelet and the cost exists
         * @return false if the destination cannot be reached
         */
        bool get_cost(lanelet::Id id, double &cost_writeback) const;

        /**
         * Get the horizon that lanelets need to be returned for, in meters.
         * 
         * @return meters
         */
        double get_current_horizon() const{
          return 100; //TODO: pull this from a param file
        }

        void load_osm_map(const std::string &file, float64_t lat, float64_t lon, float64_t alt);

        /**
         * Extracts lanelets from the OSM map and if they are the right type adds them to the road map
         */
        void parse_lanelet_element();

        /**
         * @return type of lanelet if one with matching id has already been parsed;
         *  type is currently one of "lane" or "unknown"
         */
        std::string get_primitive_type(const lanelet::Id &prim_id);

        /**
         * Translates a cad ID to a lane ID, or returns -1 if lane id not found among parsed elements.
         */
        lanelet::Id find_lane_id(const lanelet::Id &cad_id) const;

        /**
         * Euclidean distance between points 1 and 2
         */
        float64_t p2p_euclidean(const lanelet::Point3d &p1, const lanelet::Point3d &p2) const;

        /**
         * Parsing strings to lanelet ids
         */
        std::vector<lanelet::Id> lanelet_chr2num(const std::string &str) const;

        /**
         * Parsing strings to lanelet ids
         */
        std::vector<lanelet::Id> lanelet_str2num(const std::string &str) const;

        std::shared_ptr<lanelet::LaneletMap> osm_map;

        lanelet::routing::RoutingGraphUPtr routing_graph;

      private:
        std::unordered_map<lanelet::Id, lanelet::Id> road_map;
        std::unordered_map<lanelet::Id, double> cost_map;
      };
    } // namespace lanelet2_global_planner
  }   // namespace planning
} // namespace autoware

#endif // LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
