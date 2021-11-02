// Copyright 2021 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

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

      class LANELET2_GLOBAL_PLANNER_PUBLIC Lanelet2GlobalPlanner
      {
      public:
        Lanelet2GlobalPlanner() = default;

        void load_osm_map(const std::string &file, float64_t lat, float64_t lon, float64_t alt);

        /**
         * Extracts lanelets from the OSM map and if they are the right type adds them to the road map
         */
        void parse_lanelet_element();

        /**
         * Plans the optimal series of lanelets leading from the start to the end.
         * 
         * If the endpoints are not inside a lanelet the closest is used.
         * 
         * @return true if a route was found.
         */
        bool8_t plan_route(
            TrajectoryPoint &start, TrajectoryPoint &end,
            std::vector<lanelet::Id> &route) const;

        /**
         * @return type of lanelet if one with matching id has already been parsed;
         *  type is currently one of "lane" or "unknown"
         */
        std::string get_primitive_type(const lanelet::Id &prim_id);

        /**
         * Translates a cad ID to a lane ID, or returns -1 if lane id not found among parsed elements.
         */
        lanelet::Id find_lane_id(const lanelet::Id &cad_id) const;

        std::vector<lanelet::Id> get_lane_route(
            const std::vector<lanelet::Id> &from_id,
            const std::vector<lanelet::Id> &to) const;

        /**
         * Euclidean distance between points 1 and 2
         */
        float64_t p2p_euclidean(const lanelet::Point3d &p1, const lanelet::Point3d &p2) const;

        /**
         * Parsing, currently not sure what
         */
        std::vector<lanelet::Id> lanelet_chr2num(const std::string &str) const;

        /**
         * Parsing, currently not sure what
         */
        std::vector<lanelet::Id> lanelet_str2num(const std::string &str) const;

        std::shared_ptr<lanelet::LaneletMap> osm_map;

      private:
        std::vector<lanelet::Id> parking_id_list;

        std::unordered_map<lanelet::Id, lanelet::Id> road_map;
      };
    } // namespace lanelet2_global_planner
  }   // namespace planning
} // namespace autoware

#endif // LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
