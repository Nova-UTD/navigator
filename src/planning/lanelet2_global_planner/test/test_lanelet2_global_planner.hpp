// Copyright 2019 the Autoware Foundation
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

#ifndef TEST_LANELET2_GLOBAL_PLANNER_HPP_
#define TEST_LANELET2_GLOBAL_PLANNER_HPP_

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <common/types.hpp>
#include <string>
#include <memory>
#include <vector>

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include) && \
    __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <common/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using autoware::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;
using autoware::planning::lanelet2_global_planner::LaneRouteCosts;
using autoware_auto_msgs::msg::TrajectoryPoint;

class TestGlobalPlannerBaseTest : public ::testing::Test
{
public:
  // consts to map the number of encapsulating lanelets to the indices
  // of their trajectory points in origin and destination arrays
  const int NO_LANELETS = 0;
  const int ONE_LANELET = 1;
  const int MULTIPLE_LANELETS = 2;

  // there should be a route from each origin to destination lanelet
  // useful lanelet Ids to start from
  TrajectoryPoint origins[3];
  // useful lanelet IDs to end at
  TrajectoryPoint destinations[3];

  std::shared_ptr<Lanelet2GlobalPlanner> node_ptr;

  TestGlobalPlannerBaseTest(
      std::string map_file_path,
      float64_t lat,
      float64_t lon,
      float64_t alt)
  {
    node_ptr = std::make_shared<Lanelet2GlobalPlanner>();
    std::string root_folder =
        ament_index_cpp::get_package_share_directory("lanelet2_global_planner");
    // std::string file_path = std::string("/test/map_data/mapping_example_pk.osm");
    std::string file = root_folder + map_file_path;

    node_ptr->load_osm_map(file, lat, lon, alt);
    node_ptr->parse_lanelet_element();
  }

  /**
   * A collection of assertions on the output of fetch_routing_costs that
   * should hold true in any scenario
   *
   * @param costs: output vector from fetch_routing_costs
   */
  void validate_routing_costs(LaneRouteCosts costs, TrajectoryPoint destination)
  {
    // each lanelet should be included in the message only once
    // run an (inefficient) linear search for each entry
    bool failed_uniqueness = false;
    for (auto &check_pair : costs)
    {
      for (auto &search_pair : costs)
      {
        if (check_pair != search_pair)
        {
          failed_uniqueness = failed_uniqueness || (check_pair.first == search_pair.first);
        }
      }
    }
    if (failed_uniqueness)
    {
      EXPECT_FALSE(failed_uniqueness) << "Lanelet Ids should appear only once in output";
      print_route_results(costs);
    }

    // each lanelet included in the message should be able to reach the destination
    std::unordered_map<lanelet::Id, double> abs_costs; // store for later
    lanelet::Lanelets dests;
    node_ptr->find_lanelets(destination, 5, dests);
    for (auto &check_pair : costs)
    {
      double abs_cost = std::numeric_limits<double>::max();
      lanelet::ConstLanelet origin_lanelet = node_ptr->osm_map->laneletLayer.get(check_pair.first);

      for (lanelet::Lanelet dest : dests)
      {
        lanelet::Optional<lanelet::routing::Route> route = node_ptr->routing_graph->getRoute(origin_lanelet, dest);
        if (!!route)
        {
          abs_cost = std::min(abs_cost, route->length2d()); // TODO: what if cost is not length?
        }
      }
      ASSERT_TRUE(abs_cost != std::numeric_limits<double>::max())
          << "Each lanelet should have route to destination" << std::endl
          << "Lanelet ID: " << check_pair.first;

      abs_costs[check_pair.first] = abs_cost;
    }
    // The relative ordering of the costs from the message should match the relative
    // ordering of the absolute costs of the routes from those lanelets
    // more effiecient would be to sort and check, but checking every pair instead
    // only want to fail once to avoid flooding console.
    bool failed_relative_ordering = false;
    for (auto p1 : costs)
    {
      for (auto p2 : costs)
      {
        failed_relative_ordering = failed_relative_ordering || !((p1.second < p2.second) == (abs_costs[p1.first] < abs_costs[p2.first]));
      }
    }
    EXPECT_FALSE(failed_relative_ordering) << "Relative ordering of costs should remain the same";
  }

  void print_route_results(LaneRouteCosts results)
  {
    std::cerr << "Route contents: " << std::endl;
    std::cerr << "\t{ ID , COST }" << std::endl;
    for (auto lcpair : results)
    {
      std::cerr << "\t{ " << lcpair.first << " , " << lcpair.second << "}" << std::endl;
    }
  }
};

class TestGlobalPlannerMapNoCycles : public TestGlobalPlannerBaseTest
{
public:
  TestGlobalPlannerMapNoCycles() : TestGlobalPlannerBaseTest("/test/map_data/borregas_test.osm", 37.41691154328607, -122.01603989032715, 1.0)
  {
    origins[NO_LANELETS].set__x(21.71507); // above lanelet 7
    origins[NO_LANELETS].set__y(15.71280);
    origins[ONE_LANELET].set__x(21.71507); // inside lanelet 7
    origins[ONE_LANELET].set__y(4.55727);
    origins[MULTIPLE_LANELETS].set__x(-16.352); // lanelets 282, 251
    origins[MULTIPLE_LANELETS].set__y(22.959);

    destinations[NO_LANELETS].set__x(-3.4); // below lanelet 137
    destinations[NO_LANELETS].set__y(-388.8);
    destinations[ONE_LANELET].set__x(7.9156); // inside lanelet 137
    destinations[ONE_LANELET].set__y(-367.4506);
    destinations[MULTIPLE_LANELETS].set__x(-96.6244); // lanelets 402, 452, 380
    destinations[MULTIPLE_LANELETS].set__y(-338.9100);
  }
};

TEST_F(TestGlobalPlannerMapNoCycles, TestMapSetup)
{
  // TrajectoryPoint to Lanelet lookup
  lanelet::Lanelets lanelets_found;

  EXPECT_FALSE(node_ptr->find_lanelets(origins[NO_LANELETS], 5, lanelets_found)) << "Should not find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(1)) << "Should find exactly 1 nearest lanelet";
  EXPECT_EQ(lanelets_found[0].id(), lanelet::Id(7)) << "Found unexpected lanelet";
  lanelets_found.clear();

  EXPECT_FALSE(node_ptr->find_lanelets(destinations[NO_LANELETS], 5, lanelets_found)) << "Should not find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(1)) << "Should find exactly 1 nearest lanelet";
  EXPECT_EQ(lanelets_found[0].id(), lanelet::Id(137)) << "Found unexpected lanelet";
  lanelets_found.clear();

  EXPECT_TRUE(node_ptr->find_lanelets(origins[ONE_LANELET], 5, lanelets_found)) << "Should find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(1)) << "Should find exactly 1 encapsulating lanelet";
  EXPECT_EQ(lanelets_found[0].id(), lanelet::Id(7)) << "Found unexpected lanelet";
  lanelets_found.clear();

  EXPECT_TRUE(node_ptr->find_lanelets(destinations[ONE_LANELET], 5, lanelets_found)) << "Should find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(1)) << "Should find exactly 1 encapsulating lanelet";
  EXPECT_EQ(lanelets_found[0].id(), lanelet::Id(137)) << "Found unexpected lanelet";
  lanelets_found.clear();

  EXPECT_TRUE(node_ptr->find_lanelets(origins[MULTIPLE_LANELETS], 5, lanelets_found)) << "Should find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(2)) << "Should find multiple encapsulating lanelets";
  for (lanelet::Lanelet l : lanelets_found)
  {
    lanelet::Id l_id = l.id();
    EXPECT_TRUE(l_id == lanelet::Id(282) || l_id == lanelet::Id(251)) << "Found unexpected lanelet";
  }
  lanelets_found.clear();

  EXPECT_TRUE(node_ptr->find_lanelets(destinations[MULTIPLE_LANELETS], 5, lanelets_found)) << "Should find encapsulating lanelet";
  EXPECT_EQ(lanelets_found.size(), std::size_t(3)) << "Should find multiple encapsulating lanelets";
  for (lanelet::Lanelet l : lanelets_found)
  {
    lanelet::Id l_id = l.id();
    EXPECT_TRUE(l_id == lanelet::Id(402) || l_id == lanelet::Id(452) || l_id == lanelet::Id(380)) << "Found unexpected lanelet";
  }
  lanelets_found.clear();

  // Map autovalidation
  for (auto dest : destinations)
  {
    node_ptr->set_destination(dest);
    auto errors = node_ptr->routing_graph->checkValidity(false);
    for (auto error : errors)
    {
      EXPECT_TRUE(false) << "Routing graph error: " << error;
    }
  }
}

TEST_F(TestGlobalPlannerMapNoCycles, TestRoutes)
{
  // for each possible origin -> destination pair validate results
  for (auto dest : destinations)
  {
    node_ptr->set_destination(dest);
    for (auto origin : origins)
    {
      LaneRouteCosts results;
      node_ptr->fetch_routing_costs(origin, results);
      EXPECT_NE(results.size(), std::size_t(0)) << "Route should exist and costs should not be empty";
      validate_routing_costs(results, dest);
    }
  }
}

TEST_F(TestGlobalPlannerMapNoCycles, TestRouteDirectionality)
{
  // for each possible destination -> origin pair try to find route.
  // since this map has no cycles and routes should exist from origin
  // to destination, no route should exist from destinations to origin
  for (auto dest_flipped : origins)
  {
    node_ptr->set_destination(dest_flipped);
    for (auto origin_flipped : destinations)
    {
      LaneRouteCosts results;
      node_ptr->fetch_routing_costs(origin_flipped, results);
      EXPECT_EQ(results.size(), std::size_t(0)) << "Routes should follow lanelet direction";
      if (results.size() > 0)
      {
        print_route_results(results);
      }
    }
  }
}

// class TestGlobalPlannerFullMap : public ::testing::Test
// {
// public:
//   TestGlobalPlannerFullMap()
//   {
//     node_ptr = std::make_shared<Lanelet2GlobalPlanner>();
//     std::string root_folder =
//       ament_index_cpp::get_package_share_directory("lanelet2_global_planner");
//     std::string file_path = std::string("/test/map_data/AStuff_test_map.osm");
//     std::string file = root_folder + file_path;
//     float64_t lat = 37.380811523812845;
//     float64_t lon = -121.90840595108715;
//     float64_t alt = 11.0;
//     node_ptr->load_osm_map(file, lat, lon, alt);
//     node_ptr->parse_lanelet_element();
//   }
//   std::shared_ptr<Lanelet2GlobalPlanner> node_ptr;
// };

// class TestGlobalPlannerFullMapWithoutParkingSpots : public ::testing::Test
// {
// public:
//   TestGlobalPlannerFullMapWithoutParkingSpots()
//   {
//     node_ptr = std::make_shared<Lanelet2GlobalPlanner>();
//     std::string root_folder =
//       ament_index_cpp::get_package_share_directory("lanelet2_global_planner");
//     std::string file_path = std::string("/test/map_data/kashiwanoha_map.osm");
//     std::string file = root_folder + file_path;
//     float64_t lat = 35.238094905136874;
//     float64_t lon = 139.90095439549778;
//     float64_t alt = 1.0;
//     node_ptr->load_osm_map(file, lat, lon, alt);
//     node_ptr->parse_lanelet_element();
//   }
//   std::shared_ptr<Lanelet2GlobalPlanner> node_ptr;
// };

// TEST(TestFunction, Point3dCopy)
// {
//   auto assign_point3d = [](lanelet::Point3d & pcopy)
//     {
//       lanelet::Point3d p(lanelet::utils::getId(), 2.0, 4.0, 6.0);
//       pcopy = p;
//     };

//   lanelet::Point3d point3d;
//   assign_point3d(point3d);
//   ASSERT_DOUBLE_EQ(2.0, point3d.x());
//   ASSERT_DOUBLE_EQ(4.0, point3d.y());
//   ASSERT_DOUBLE_EQ(6.0, point3d.z());
// }

// TEST(TestFunction, Point3dEigen)
// {
//   lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
//   lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
//   Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
//   Eigen::Vector3d pd2 = pd.array().square();
//   float64_t dist = std::sqrt(pd2.x() + pd2.y() + pd2.z());
//   float64_t actual = std::sqrt(
//     std::pow(p2.x() - p1.x(), 2) +
//     std::pow(p2.y() - p1.y(), 2) +
//     std::pow(p2.z() - p1.z(), 2));
//   ASSERT_DOUBLE_EQ(actual, dist);
// }

// TEST(TestFunction, Point3dMean)
// {
//   lanelet::Point3d p1(lanelet::utils::getId(), 1, 4, 7);
//   lanelet::Point3d p2(lanelet::utils::getId(), 2, 5, 8);
//   lanelet::Point3d p3(lanelet::utils::getId(), 3, 6, 9);
//   lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
//   size_t num_points = ls.size();
//   // sum x,y,z points
//   float64_t mean_x = 0.0;
//   float64_t mean_y = 0.0;
//   float64_t mean_z = 0.0;
//   std::for_each(
//     ls.begin(), ls.end(), [&](lanelet::Point3d p)
//     {
//       mean_x += p.x() / static_cast<float64_t>(num_points);
//       mean_y += p.y() / static_cast<float64_t>(num_points);
//       mean_z += p.z() / static_cast<float64_t>(num_points);
//     });
//   ASSERT_DOUBLE_EQ(2.0, mean_x);
//   ASSERT_DOUBLE_EQ(5.0, mean_y);
//   ASSERT_DOUBLE_EQ(8.0, mean_z);
// }

// TEST_F(TestGlobalPlannerBasicMap, TestFindParking)
// {
//   // Find parking id from point
//   // parking id: 5830 location: 20.7607, -10.2697, 15.6 near_roads: 479054
//   lanelet::Point3d position(lanelet::utils::getId(), 20.7607, -10.2697, 15.6);
//   lanelet::Id parking_id = node_ptr->find_nearparking_from_point(position);
//   EXPECT_EQ(parking_id, 5830);

//   // Compute parking centre
//   lanelet::Point3d p1;
//   bool8_t ret = false;
//   ret = node_ptr->compute_parking_center(parking_id, p1);
//   EXPECT_TRUE(ret);

//   // near road id
//   lanelet::Id near_road = node_ptr->find_nearroute_from_parking(parking_id);
//   EXPECT_EQ(near_road, 479054);
// }

// TEST_F(TestGlobalPlannerBasicMap, TestP2pDistance)
// {
//   lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
//   lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
//   float64_t dist = node_ptr->p2p_euclidean(p1, p2);
//   float64_t actual = std::sqrt(
//     std::pow(p2.x() - p1.x(), 2) +
//     std::pow(p2.y() - p1.y(), 2) +
//     std::pow(p2.z() - p1.z(), 2));
//   ASSERT_DOUBLE_EQ(actual, dist);
// }

// TEST_F(TestGlobalPlannerBasicMap, TestLanesChr2num)
// {
//   std::string str = "[u'429933', u'430462']";
//   std::vector<lanelet::Id> num = node_ptr->lanelet_chr2num(str);
//   ASSERT_EQ(num[0], 429933);
//   ASSERT_EQ(num[1], 430462);
// }

// TEST_F(TestGlobalPlannerBasicMap, TestLanesStr2num)
// {
//   std::string str = "1258,4258,3798";
//   std::vector<lanelet::Id> num = node_ptr->lanelet_str2num(str);
//   ASSERT_EQ(num.size(), 3U);
//   ASSERT_EQ(num[0], 1258);
//   ASSERT_EQ(num[1], 4258);
//   ASSERT_EQ(num[2], 3798);
// }

// TEST_F(TestGlobalPlannerFullMap, TestFindParkingaccess)
// {
//   lanelet::Id parking_id = 8008;
//   lanelet::Id expect_parking_access_id = 9538;
//   lanelet::Id parking_access_id = node_ptr->find_parkingaccess_from_parking(parking_id);
//   EXPECT_EQ(parking_access_id, expect_parking_access_id);
//   parking_id = 8429;
//   expect_parking_access_id = 8308;
//   parking_access_id = node_ptr->find_parkingaccess_from_parking(parking_id);
//   EXPECT_EQ(parking_access_id, expect_parking_access_id);
//   parking_id = 9552;
//   expect_parking_access_id = 9140;
//   parking_access_id = node_ptr->find_parkingaccess_from_parking(parking_id);
//   EXPECT_EQ(parking_access_id, expect_parking_access_id);
//   parking_id = 7663;
//   expect_parking_access_id = 9140;
//   parking_access_id = node_ptr->find_parkingaccess_from_parking(parking_id);
//   EXPECT_EQ(parking_access_id, expect_parking_access_id);
// }

// TEST_F(TestGlobalPlannerFullMap, TestFindNoParkingaccess)
// {
//   lanelet::Id parking_id = 8220;
//   lanelet::Id expect_parking_access_id = -1;
//   lanelet::Id parking_access_id = node_ptr->find_parkingaccess_from_parking(parking_id);
//   EXPECT_EQ(parking_access_id, expect_parking_access_id);
// }

// TEST_F(TestGlobalPlannerFullMap, TestFindLane)
// {
//   lanelet::Id parking_access_id = 7849;
//   std::vector<lanelet::Id> lane_id = node_ptr->find_lane_from_parkingaccess(parking_access_id);
//   EXPECT_EQ(lane_id, (std::vector<lanelet::Id>{6518, 6525}));

//   parking_access_id = 8308;
//   lane_id = node_ptr->find_lane_from_parkingaccess(parking_access_id);
//   EXPECT_EQ(lane_id, (std::vector<lanelet::Id>{6399, 6392}));

//   parking_access_id = 9140;
//   lane_id = node_ptr->find_lane_from_parkingaccess(parking_access_id);
//   EXPECT_EQ(lane_id, (std::vector<lanelet::Id>{6392, 6399}));
// }

// // test find route giving a single lane option
// TEST_F(TestGlobalPlannerFullMap, TestFindRouteSingleLane)
// {
//   std::vector<lanelet::Id> start_lane_id;
//   std::vector<lanelet::Id> end_lane_id;
//   std::vector<lanelet::Id> route_id;
//   // entry route
//   start_lane_id = std::vector<lanelet::Id>{6714};
//   end_lane_id = std::vector<lanelet::Id>{6546};
//   route_id = node_ptr->get_lane_route(start_lane_id, end_lane_id);
//   EXPECT_GT(route_id.size(), 0u);

//   // designate drop-off 1 to parking route
//   start_lane_id = std::vector<lanelet::Id>{6392};
//   end_lane_id = std::vector<lanelet::Id>{6518};
//   // result: 6392, 7319, 6686, 6672, 6504, 6560, 6742, 7387, 6518
//   route_id = node_ptr->get_lane_route(start_lane_id, end_lane_id);
//   EXPECT_GT(route_id.size(), 0u);

//   // designate drop-off 2 to parking route
//   start_lane_id = start_lane_id = std::vector<lanelet::Id>{6399};
//   end_lane_id = start_lane_id = std::vector<lanelet::Id>{6490};
//   // result: 6399, 6826, 6616, 6618, 6546, 6378, 6280, 6490
//   route_id = node_ptr->get_lane_route(start_lane_id, end_lane_id);

//   // parking to designate pick_up 1 route
//   start_lane_id = start_lane_id = std::vector<lanelet::Id>{6525};
//   end_lane_id = start_lane_id = std::vector<lanelet::Id>{6399};
//   // 6525, 7251, 6749, 6567, 6511, 6679, 6693, 7013, 6399
//   route_id = node_ptr->get_lane_route(start_lane_id, end_lane_id);
//   EXPECT_GT(route_id.size(), 0u);
// }

// // test find route giving a multiple lane option (so can find route any direction)
// TEST_F(TestGlobalPlannerFullMap, TestFindRouteMutipleLanes)
// {
//   std::vector<lanelet::Id> start_lane_id;
//   std::vector<lanelet::Id> end_lane_id;
//   std::vector<lanelet::Id> route_id;
//   start_lane_id = std::vector<lanelet::Id>{6546, 6553};
//   end_lane_id = std::vector<lanelet::Id>{6518, 6525};
//   route_id = node_ptr->get_lane_route(start_lane_id, end_lane_id);
//   EXPECT_GT(route_id.size(), 0u);
// }

// TEST_F(TestGlobalPlannerFullMap, TestFindParkingFromPoint)
// {
//   // Vehicle location in the map frame: -25.9749 102.129 -1.74268
//   // parking id: 101930
//   lanelet::Point3d position_1(lanelet::utils::getId(), -25.97, 102.12, -1.74);
//   lanelet::Id parking_id_1 = node_ptr->find_nearparking_from_point(position_1);
//   EXPECT_EQ(parking_id_1, 101930);

//   // Vehicle location in the map frame: -12.5381 67.6976 -1.34077
//   // parking id: 8113
//   lanelet::Point3d position_2(lanelet::utils::getId(), -12.53, 67.69, -1.34);
//   lanelet::Id parking_id_2 = node_ptr->find_nearparking_from_point(position_2);
//   EXPECT_EQ(parking_id_2, 8113);
// }

// TEST_F(TestGlobalPlannerFullMap, TestPlanFullRoute)
// {
//   // take the parking spot from previous test
//   autoware_auto_msgs::msg::TrajectoryPoint from_point;
//   from_point.x = -25.97;
//   from_point.y = 102.12;
//   autoware_auto_msgs::msg::TrajectoryPoint to_point;
//   to_point.x = -12.53;
//   to_point.y = 67.69;
//   std::vector<lanelet::Id> route;
//   bool8_t result = node_ptr->plan_route(from_point, to_point, route);
//   EXPECT_TRUE(result);
// }

// TEST_F(TestGlobalPlannerFullMapWithoutParkingSpots, TestPlanFullRouteWithoutParkingSpots)
// {
//   autoware_auto_msgs::msg::TrajectoryPoint from_point;
//   from_point.x = -25.97;
//   from_point.y = 102.12;
//   autoware_auto_msgs::msg::TrajectoryPoint to_point;
//   to_point.x = -12.53;
//   to_point.y = 67.69;
//   std::vector<lanelet::Id> route;
//   bool8_t result = node_ptr->plan_route(from_point, to_point, route);
//   EXPECT_TRUE(result);
// }
//
// };
#endif // TEST_LANELET2_GLOBAL_PLANNER_HPP_
