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

// TODO for this file: 
//  - make sure that get_routing_costs checks for cost map and routing graph
//  - improve horizon function and general configurability
//  - work in non-default cost configurables for both routing cost and horizon calculations

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

using namespace lanelet::routing;

namespace autoware
{
  namespace planning
  {
    namespace lanelet2_global_planner
    {

      Lanelet2GlobalPlanner::Lanelet2GlobalPlanner(): cost_map(){}

      void Lanelet2GlobalPlanner::load_osm_map(
          const std::string &file,
          float64_t lat, float64_t lon, float64_t alt)
      {
        if (osm_map){
          osm_map.reset();
        }

        osm_map = load(file, lanelet::projection::UtmProjector(lanelet::Origin({lat, lon, alt})));

        // throw map load error
        if (!osm_map){
          throw std::runtime_error("Lanelet2GlobalPlanner: Map load fail");
        }
      }

      void Lanelet2GlobalPlanner::parse_lanelet_element(){
        if (osm_map){
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

      bool Lanelet2GlobalPlanner::find_lanelets(TrajectoryPoint location, uint depth, std::vector<lanelet::Lanelet> &result_wb) const{
        const lanelet::Point2d point(lanelet::utils::getId(), location.x, location.y, 0.0);
        auto nearby_lanelets = osm_map->laneletLayer.nearest(point, depth);

        if (nearby_lanelets.empty())
        {
          return false;
        }
        else
        {
          bool flag_found_containing = false;
          // only want the closest lanelet, and lanelet2 doesn't provide in order
          // do linear search on lanelets if containing not found
          lanelet::Lanelet nearest_lanelet;
          double nearest_distance = std::numeric_limits<double>::max();

          for (const auto &lanelet : nearby_lanelets)
          {
            if(lanelet::geometry::inside(lanelet, point)){
              // Add to results and update flag
              flag_found_containing = true;
              result_wb.push_back(lanelet);
            } else {
              // check if it's the nearest
              double cur_dist = lanelet::geometry::distance2d(lanelet.polygon2d(), point);
              if (cur_dist < nearest_distance)
              {
                nearest_lanelet = lanelet;
                nearest_distance = cur_dist;
              }
            }
          }

          if(!flag_found_containing){
            result_wb.push_back(nearest_lanelet);
          }

          return flag_found_containing;
        }
      }


      /**
         * Fetches lanelets near current location and labels them with routing cost.
         * 
         * If the location is not inside a lanelet the closest is used.
         * 
         * Writes <Id, cost> pairs to a new vector passed back to provided address
         */
        void Lanelet2GlobalPlanner::fetch_routing_costs(TrajectoryPoint &location, LaneRouteCosts &costs) const {
          // using a map initially since it is possible a lane might be added multiple times
          std::map<lanelet::Id, double> message_cost_map;
          
          std::vector<lanelet::Lanelet> origin_lanelets;
          find_lanelets(location, 5, origin_lanelets);

          double horizon = get_current_horizon();

          // find lanelets within horizon and add them to the message if 
          // routing costs exists
          for(lanelet::Lanelet l : origin_lanelets){
            double cur_cost;
            if(get_cost(l.id(), cur_cost)){
              // only add to message if lanelets can reach destination
              lanelet::ConstLanelets reachable = routing_graph->reachableSet(l, horizon);
              for(lanelet::ConstLanelet r : reachable){
                lanelet::Id r_id = r.id();
                double r_cost;
                if(get_cost(r_id, r_cost)){
                  // want cost relative to current position
                  r_cost = r_cost - cur_cost;
                  // only add to message if lanelets can reach destination
                  // it is possible that if our origin position included multiple 
                  // lanelets that r already has a cost. We want to keep the lowest.
                  auto msg_cost_search = message_cost_map.find(r_id);
                  if(msg_cost_search == message_cost_map.end()){
                    message_cost_map.emplace(r_id, r_cost);
                  } else{
                    message_cost_map[r_id] = std::min(r_cost, message_cost_map[r_id]);
                  }
                }
              }
            }            
          }

          // from message map extract all pairs and put to output vector
          for(auto map_pair : message_cost_map){
            LaneRouteCost cost_pair;
            cost_pair.first = map_pair.first;
            cost_pair.second= map_pair.second;
            costs.push_back(cost_pair);
          }

        }

        bool Lanelet2GlobalPlanner::get_cost(lanelet::Id id, double &cost_wb) const{
          auto search_res = cost_map.find(id);
          if(search_res == cost_map.end()){
            // no route from given lane to destination exists
            return false;
          }else{
            // route exists. Update cost and return
            cost_wb = search_res->second;
            return true;
          }
        }

        /* WARNING: this current code is slow and is not meant to be used
         * while the car is in operation. It is designed for when the whole
         * map is loaded at once and provides rapid lookup for lane costs.
         */
        void Lanelet2GlobalPlanner::set_destination(TrajectoryPoint &goal) {
          // Visit each lane that can lead to the destination point and add 
          // its routing cost to the map

          cost_map.clear();

          // Create routing graph with traffic rules
          lanelet::traffic_rules::TrafficRulesPtr trafficRules =
              lanelet::traffic_rules::TrafficRulesFactory::create(
                  lanelet::Locations::Germany, // TODO: figure out if its safe to change locations
                  lanelet::Participants::Vehicle);

          routing_graph = RoutingGraph::build(*osm_map, *trafficRules);

          std::vector<lanelet::Lanelet> containing_lanelets;
          find_lanelets(goal, 5, containing_lanelets);

          LaneletVisitFunction visit_func = [=](const LaneletVisitInformation &info)->bool {
              // each lanelet should only be traversed once with its lowest cost, so we 
              // shouldn't have to worry about double visits
              cost_map.emplace(info.lanelet.id(), info.cost);
              return true;
            };

          for(lanelet::Lanelet &dest_lanelet : containing_lanelets){
            // forEachPredecessor with lane changes allowed *should* automatically
            // handle adjacent but non-preceeding relations. Using default cost for now
            routing_graph->forEachPredecessor(dest_lanelet, visit_func);
          }
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
