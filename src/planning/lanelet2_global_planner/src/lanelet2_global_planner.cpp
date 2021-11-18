#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <lanelet2_core/geometry/Lanelet.h>

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

using std::vector;

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

using lanelet::Lanelet;
using lanelet::routing::Route;
using lanelet::routing::LaneletPath;
using lanelet::routing::RoutingGraph;
using lanelet::routing::LaneletRelation;
using lanelet::routing::LaneletRelations;

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
          TrajectoryPoint &end_point, 
          vector<lanelet::Id> &route, 
          vector<lanelet::Id> &lane_changes) const
      {

        // Note that for this function there is a terminology mismatch with the rest of nova:
        // For lanelet, route means the roads to take (with possible lane changes) while path means 
        // the lanelets to take. For nova, route referes to the lanelets to take while path means 
        // trajectory of the vehicle. So a lanelet path is the same as a nova route.

        const lanelet::Point3d start(lanelet::utils::getId(), start_point.x, start_point.y, 0.0);
        const lanelet::Point3d end(lanelet::utils::getId(), end_point.x, end_point.y, 0.0);

        const lanelet::Point2d start2d(start.id(), start.x(), start.y());
        const lanelet::Point2d end2d(end.id(), end.x(), end.y());

        vector<Lanelet> start_lanes;
        vector<Lanelet> end_lanes;

        // Find the origin lanelet

        // Grab all the lanes that encapsulate the vehicles position (up to 5) OR if none are found 
        // the lane closes to the vehicle. 
        //TODO: (eganj) Check that lanes are running in the same direction as the vehicle

        auto nearest_lanelets_start = osm_map->laneletLayer.nearest(start, 5);
        if (nearest_lanelets_start.empty())
        {
          std::cerr << "Couldn't find nearest lanelet to start." << std::endl;
        }
        else
        {
          // Track lanelet with centerline closest to start point in case start point is not inside any lanes
          Lanelet nearest_candidate;
          double dist_to_nearest = std::numeric_limits<double>().max();

          for(Lanelet start_candidate : nearest_lanelets_start){
            if(lanelet::geometry::inside(start_candidate, start2d)){
              start_lanes.push_back(start_candidate);
            }else{
              double dist_to_candidate = lanelet::geometry::distanceToCenterline2d(start_candidate, start2d);
              if(dist_to_candidate < dist_to_nearest){
                nearest_candidate = start_candidate;
                dist_to_nearest = dist_to_candidate;
              }
            }

            if(start_lanes.empty()){
              // use closest guess instead of encapsulating lanelets
              // TODO: report to console or safety manager?
              start_lanes.push_back(nearest_candidate);
            }
          }
        }

        auto nearest_lanelets_end = osm_map->laneletLayer.nearest(end, 5);
        if (nearest_lanelets_end.empty())
        {
          std::cerr << "Couldn't find nearest lanelet to goal." << std::endl;
        }
        else
        {
          // Track lanelet with centerline closest to start point in case start point is not inside any lanes
          Lanelet nearest_candidate;
          double dist_to_nearest = std::numeric_limits<double>().max();

          for (Lanelet end_candidate : nearest_lanelets_end){
            if (lanelet::geometry::inside(end_candidate, end2d)) {
              end_lanes.push_back(end_candidate);
            }
            else {
              double dist_to_candidate = lanelet::geometry::distanceToCenterline2d(end_candidate, end2d);
              if (dist_to_candidate < dist_to_nearest) {
                nearest_candidate = end_candidate;
                dist_to_nearest = dist_to_candidate;
              }
            }
          }

           if (end_lanes.empty()) {
              // use closest guess instead of encapsulating lanelets
              // TODO: report to console or safety manager?
              end_lanes.push_back(nearest_candidate);
            }
        }

        // plan a route using lanelet2 lib
        lanelet::Optional<Route> lanelet_route;
        lanelet::routing::LaneletPath lanelet_path;
        get_lane_route(start_lanes, end_lanes, lanelet_route, lanelet_path);

        // TODO: Route, Path to vectors
        route = vector<lanelet::Id>();
        lane_changes = vector<lanelet::Id>();

        // There is unfortunately no way to ask the route for it's lanelets directly so
        // we must traverse the route ourselves
        for(lanelet::ConstLanelet path_segment: lanelet_path){
          // add the segment to the route
          route.push_back(path_segment.id());

          LaneletRelations lane_change_options_left = lanelet_route->leftRelations(path_segment);
          LaneletRelations lane_change_options_right = lanelet_route->leftRelations(path_segment);

          for(LaneletRelation l_option : lane_change_options_left){
            lane_changes.push_back(l_option.lanelet.id());
          }
          for(LaneletRelation r_option : lane_change_options_right){
            lane_changes.push_back(r_option.lanelet.id());
          }
          
        }

        // Order of the lane changes doesn't matter but should be unique
        std::sort(lane_changes.begin(), lane_changes.end());
        std::unique(lane_changes.begin(), lane_changes.end());

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
      float64_t Lanelet2GlobalPlanner::get_lane_route(
          const vector<Lanelet> &from_lanes, const vector<Lanelet> &to_lanes, 
          lanelet::Optional<Route> &route_found, lanelet::routing::LaneletPath &path_found) const
      {

        // Create routing graph with traffic rules
        lanelet::traffic_rules::TrafficRulesPtr trafficRules =
            lanelet::traffic_rules::TrafficRulesFactory::create(
                lanelet::Locations::Germany, //TODO: figure out if its safe to change locations
                lanelet::Participants::Vehicle);

        lanelet::routing::RoutingGraphUPtr routingGraph =
            lanelet::routing::RoutingGraph::build(*osm_map, *trafficRules);

        // tracker for linear search of shortest route
        float64_t route_length = std::numeric_limits<float64_t>().max();

        // We have multiple lane candidates that all encapsulate the endpoints.
        // Different selections of lanelets may have different lengths of routes; we want the shortest
        for(Lanelet from_lane : from_lanes){
          for(Lanelet to_lane : to_lanes){
            lanelet::Optional<Route> route_candidate = routingGraph->getRoute(from_lane, to_lane);
            if (route_candidate && route_candidate->length2d() < route_length){
              route_found = route_candidate;
              path_found = route_candidate->shortestPath();
            }
          }
        }

        return route_length;
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
