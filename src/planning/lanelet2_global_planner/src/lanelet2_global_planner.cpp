#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <lanelet2_core/geometry/Area.h>

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace autoware
{
  namespace planning
  {
    namespace lanelet2_global_planner
    {

      void Lanelet2GlobalPlanner::load_osm_map(
          const std::string &file,
          float64_t lat, float64_t lon, float64_t alt)
      {
        if (osm_map)
        {
          osm_map.reset();
        }
        osm_map = load(
            file, lanelet::projection::UtmProjector(
                      lanelet::Origin({lat, lon, alt})));

        // throw map load error
        if (!osm_map)
        {
          throw std::runtime_error("Lanelet2GlobalPlanner: Map load fail");
        }
      }

      void Lanelet2GlobalPlanner::parse_lanelet_element()
      {
        if (osm_map)
        {
          // parsing lanelet layer
          typedef std::unordered_map<lanelet::Id, lanelet::Id>::iterator it_lane;
          std::pair<it_lane, bool8_t> result_lane;
          for (const auto &lanelet : osm_map->laneletLayer)
          {
            // filter-out non road type
            if (lanelet.hasAttribute("subtype") &&
                lanelet.hasAttribute("cad_id") &&
                lanelet.attribute("subtype") == "road")
            {
              lanelet::Id lane_id = lanelet.id();
              lanelet::Id lane_cad_id = *lanelet.attribute("cad_id").asId();
              result_lane = road_map.emplace(lane_cad_id, lane_id);
              if (!result_lane.second)
              {
                throw std::runtime_error("Lanelet2GlobalPlanner: Parsing osm lane from map fail");
              }
            }
          }
        }
      }

      bool8_t Lanelet2GlobalPlanner::plan_route(
          TrajectoryPoint &start_point,
          TrajectoryPoint &end_point, std::vector<lanelet::Id> &route) const
      {

        // Note that for this function there is a terminology mismatch with the rest of nova:
        // For lanelet, route means the roads to take (with possible lane changes) while path means 
        // the lanelets to take. For nova, route referes to the lanelets to take while path means 
        // trajectory of the vehicle. So a lanelet path is the same as a nova route.

        const lanelet::Point3d start(lanelet::utils::getId(), start_point.x, start_point.y, 0.0);
        const lanelet::Point3d end(lanelet::utils::getId(), end_point.x, end_point.y, 0.0);

        lanelet::Lanelet lane_start;
        lanelet::Lanelet lane_end;

        // Find the origin lanelet

        // Two nearest lanelets should, in theory, be the lanelets for each direction
        // TODO: (eganj) verify that the right lanelets have been found (car is facing in direction of lane) and if not alert safety
        auto nearest_lanelets_start = osm_map->laneletLayer.nearest(start, 1);
        if (nearest_lanelets_start.empty())
        {
          std::cerr << "Couldn't find nearest lanelet to start." << std::endl;
        }
        else
        {
          lane_start = *(nearest_lanelets_start.begin());
        }

        // find the destination lanelet
        // Two nearest lanelets should, in theory, be the lanelets for each direction
        auto nearest_lanelets_end = osm_map->laneletLayer.nearest(end, 1);
        if (nearest_lanelets_end.empty())
        {
          std::cerr << "Couldn't find nearest lanelet to goal." << std::endl;
        }
        else
        {
          lane_end = *(nearest_lanelets_end.begin());
        }

        // plan a route using lanelet2 lib
        lanelet::routing::Route lanelet_route;
        lanelet::routing::LaneletPath lanelet_path;
        
        get_lane_route(lane_start, lane_end, lanelet_route, lanelet_path);

        // TODO: fix temp hack, put route in as the path to 

        return route.size() > 0;
      }

      std::string Lanelet2GlobalPlanner::get_primitive_type(const lanelet::Id &prim_id)
      {
        if (osm_map->laneletLayer.exists(prim_id))
        {
          return "lane";
        }
        else
        {
          return "unknown";
        }
      }

      lanelet::Id Lanelet2GlobalPlanner::find_lane_id(const lanelet::Id &cad_id) const
      {
        lanelet::Id lane_id = -1;
        // search the map
        auto it = road_map.find(cad_id);
        if (it != road_map.end())
        {
          // pick the first near road (this version only give the first one for now)
          lane_id = it->second;
        }
        return lane_id;
      }

      /**
       * @brief Plan a route between the two lane ids given. Pass route and path by reference.
       * 
       * @param from_id 
       * @param to_id 
       * @return double length of planned route or -1 if route not found
       */
      double Lanelet2GlobalPlanner::get_lane_route(
          const lanelet::Lanelet &from_lane, const lanelet::Lanelet &to_lane, 
          lanelet::routing::Route &route_found, lanelet::routing::LaneletPath &path_found) const
      {
        lanelet::traffic_rules::TrafficRulesPtr trafficRules =
            lanelet::traffic_rules::TrafficRulesFactory::create(
                lanelet::Locations::Germany, //TODO: figure out if its safe to change locations
                lanelet::Participants::Vehicle);
        lanelet::routing::RoutingGraphUPtr routingGraph =
            lanelet::routing::RoutingGraph::build(*osm_map, *trafficRules);

        route_found = *(routingGraph->getRoute(from_lane, to_lane));            // TODO: does this break if no route found?

        if (true) //TODO
        {
          path_found = route_found.shortestPath();
          return route_found.length2d();
        }
        else
        {
          // TODO: route not found
        }

        return -1;
      }

      /**
      * Fancy autoware pythagorean theorem
      */
      float64_t Lanelet2GlobalPlanner::p2p_euclidean(
          const lanelet::Point3d &p1,
          const lanelet::Point3d &p2) const
      {
        Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
        Eigen::Vector3d pd2 = pd.array().square();
        return std::sqrt(pd2.x() + pd2.y() + pd2.z());
      }

      std::vector<lanelet::Id> Lanelet2GlobalPlanner::lanelet_chr2num(const std::string &str) const
      {
        // expecting e.g. str = "[u'429933', u'430462']";
        // extract number at 3-8, 14-19
        std::string prefix_str = "'";
        size_t pos = 0U;
        size_t counter = 0U;
        size_t start = 0U;
        size_t end = 0U;
        std::vector<lanelet::Id> lanes;
        while ((pos = str.find(prefix_str, pos)) != std::string::npos)
        {
          ++counter;
          if (counter % 2 == 0U)
          {
            end = pos;
            std::string num_str = str.substr(start + 1, end - start - 1);
            lanelet::Id num_id = static_cast<lanelet::Id>(std::atoi(num_str.c_str()));
            lanes.push_back(num_id);
          }
          else
          {
            start = pos;
          }
          pos++;
        }
        return lanes;
      }

      std::vector<lanelet::Id> Lanelet2GlobalPlanner::lanelet_str2num(const std::string &str) const
      {
        // expecting no space comma e.g. str = "1523,4789,4852";
        std::vector<lanelet::Id> result_nums;
        std::regex delimiter(",");
        std::sregex_token_iterator first{str.begin(), str.end(), delimiter, -1}, last;
        std::vector<std::string> tokens{first, last};
        for (auto t : tokens)
        {
          lanelet::Id num_id = static_cast<lanelet::Id>(std::atoi(t.c_str()));
          result_nums.emplace_back(num_id);
        }
        return result_nums;
      }
    } // namespace lanelet2_global_planner
  }   // namespace planning
} // namespace autoware
