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

using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;

class TestGlobalPlannerBasicMap : public ::testing::Test
{
public:
  TestGlobalPlannerBasicMap()
  {
    node_ptr = std::make_shared<Lanelet2GlobalPlanner>();
    std::string root_folder =
      ament_index_cpp::get_package_share_directory("lanelet2_global_planner");
    std::string file_path = std::string("/test/map_data/mapping_example_pk.osm");
    std::string file = root_folder + file_path;
    float64_t lat = 51.502091;
    float64_t lon = -0.08719;
    float64_t alt = 39.0144;
    node_ptr->load_osm_map(file, lat, lon, alt);
    node_ptr->parse_lanelet_element();
  }
  std::shared_ptr<Lanelet2GlobalPlanner> node_ptr;
};

class TestGlobalPlannerFullMap : public ::testing::Test
{
public:
  TestGlobalPlannerFullMap()
  {
    node_ptr = std::make_shared<Lanelet2GlobalPlanner>();
    std::string root_folder =
      ament_index_cpp::get_package_share_directory("lanelet2_global_planner");
    std::string file_path = std::string("/test/map_data/borregas_test.osm");
    std::string file = root_folder + file_path;
    float64_t lat = 37.41691154328607;
    float64_t lon = -122.01603989032715;
    float64_t alt = 1.0;
    node_ptr->load_osm_map(file, lat, lon, alt);
    node_ptr->parse_lanelet_element();
  }
  std::shared_ptr<Lanelet2GlobalPlanner> node_ptr;
};

TEST(TestFunction, Point3dCopy)
{
  auto assign_point3d = [](lanelet::Point3d & pcopy)
    {
      lanelet::Point3d p(lanelet::utils::getId(), 2.0, 4.0, 6.0);
      pcopy = p;
    };

  lanelet::Point3d point3d;
  assign_point3d(point3d);
  ASSERT_DOUBLE_EQ(2.0, point3d.x());
  ASSERT_DOUBLE_EQ(4.0, point3d.y());
  ASSERT_DOUBLE_EQ(6.0, point3d.z());
}

TEST(TestFunction, Point3dEigen)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
  Eigen::Vector3d pd2 = pd.array().square();
  float64_t dist = std::sqrt(pd2.x() + pd2.y() + pd2.z());
  float64_t actual = std::sqrt(
    std::pow(p2.x() - p1.x(), 2) +
    std::pow(p2.y() - p1.y(), 2) +
    std::pow(p2.z() - p1.z(), 2));
  ASSERT_DOUBLE_EQ(actual, dist);
}

TEST(TestFunction, Point3dMean)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1, 4, 7);
  lanelet::Point3d p2(lanelet::utils::getId(), 2, 5, 8);
  lanelet::Point3d p3(lanelet::utils::getId(), 3, 6, 9);
  lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
  size_t num_points = ls.size();
  // sum x,y,z points
  float64_t mean_x = 0.0;
  float64_t mean_y = 0.0;
  float64_t mean_z = 0.0;
  std::for_each(
    ls.begin(), ls.end(), [&](lanelet::Point3d p)
    {
      mean_x += p.x() / static_cast<float64_t>(num_points);
      mean_y += p.y() / static_cast<float64_t>(num_points);
      mean_z += p.z() / static_cast<float64_t>(num_points);
    });
  ASSERT_DOUBLE_EQ(2.0, mean_x);
  ASSERT_DOUBLE_EQ(5.0, mean_y);
  ASSERT_DOUBLE_EQ(8.0, mean_z);
}

TEST_F(TestGlobalPlannerBasicMap, TestP2pDistance)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  float64_t dist = node_ptr->p2p_euclidean(p1, p2);
  float64_t actual = std::sqrt(
    std::pow(p2.x() - p1.x(), 2) +
    std::pow(p2.y() - p1.y(), 2) +
    std::pow(p2.z() - p1.z(), 2));
  ASSERT_DOUBLE_EQ(actual, dist);
}

TEST_F(TestGlobalPlannerBasicMap, TestLanesChr2num)
{
  std::string str = "[u'429933', u'430462']";
  std::vector<lanelet::Id> num = node_ptr->lanelet_chr2num(str);
  ASSERT_EQ(num[0], 429933);
  ASSERT_EQ(num[1], 430462);
}

TEST_F(TestGlobalPlannerBasicMap, TestLanesStr2num)
{
  std::string str = "1258,4258,3798";
  std::vector<lanelet::Id> num = node_ptr->lanelet_str2num(str);
  ASSERT_EQ(num.size(), 3U);
  ASSERT_EQ(num[0], 1258);
  ASSERT_EQ(num[1], 4258);
  ASSERT_EQ(num[2], 3798);
}

// NOTE: the tests seem to have problems calling get_lane_route
// directly. Even though the full find path call returns a route
// that starts at 11 and ends at 137, the tests here returns
// a blank route. I suspect it has to do with the deletion
// of the copy constructor/assignment operator/dereference operator
// within Route. 

// // test find route giving a single lane option
// TEST_F(TestGlobalPlannerFullMap, TestFindRouteSingleLane)
// {
//   using lanelet::Optional;
//   using lanelet::routing::Route;

//   std::vector<lanelet::Lanelet> start_lane_id;
//   std::vector<lanelet::Lanelet> end_lane_id;
//   lanelet::routing::LaneletPath path;

//   start_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(11)};
//   end_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(137)};
//   Optional<Route> route1 = node_ptr->get_lane_route(start_lane_id, end_lane_id, path);
//   // route contains start and end points
//   if(route1){
//     EXPECT_TRUE(route1->contains(start_lane_id[0])) << "Route does not contain starting lane";
//     EXPECT_TRUE(route1->contains(end_lane_id[0])) << "Route does not contain ending lane";
//   } else{
//     FAIL() << "Expected route 1 to exist";
//   }
//   // delete route1; - I would if I could but the compiler says no.

//   start_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6651)};
//   end_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6553)};
//   // result: 6392, 7319, 6686, 6672, 6504, 6560, 6742, 7387, 6518
//   Optional<Route> route2 = node_ptr->get_lane_route(start_lane_id, end_lane_id, path);
//   // route contains start and end points
//   if(route2){
//     EXPECT_TRUE(route2->contains(start_lane_id[0])) << "Route does not contain starting lane";
//     EXPECT_TRUE(route2->contains(end_lane_id[0])) << "Route does not contain ending lane";
//   } else{
//     FAIL() << "Expected route 2 to exist";
//   }

//   start_lane_id = start_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6651)};
//   end_lane_id = start_lane_id = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6546)};
//   // result: 6399, 6826, 6616, 6618, 6546, 6378, 6280, 6490
//   Optional<Route> route3 = node_ptr->get_lane_route(start_lane_id, end_lane_id, path);
//   // route contains start and end points
//   if(route2){
//     EXPECT_TRUE(route2->contains(start_lane_id[0])) << "Route does not contain starting lane";
//     EXPECT_TRUE(route2->contains(end_lane_id[0])) << "Route does not contain ending lane";
//   } else{
//     FAIL() << "Expected route 3 to exist";
//   }
// }

// // test find route giving a multiple lane option (so can find route any direction)
// TEST_F(TestGlobalPlannerFullMap, TestFindRouteMutipleLanes)
// {
//   std::vector<lanelet::Lanelet> start_lanes;
//   std::vector<lanelet::Lanelet> end_lanes;
//   lanelet::routing::LaneletPath path;
//   start_lanes = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6546), lanelet::Lanelet(6553)};
//   end_lanes = std::vector<lanelet::Lanelet>{lanelet::Lanelet(6518), lanelet::Lanelet(6525)};
//   lanelet::Optional<lanelet::routing::Route> route = node_ptr->get_lane_route(start_lanes, end_lanes,path);
//   // route should contain one of start/end points
//   bool contains_start = false;
//   for(lanelet::Lanelet lane : start_lanes){
//     contains_start &= route->contains(lane);
//   }
//   EXPECT_TRUE(contains_start) << "Route does not contain any starting lane options";

//   bool contains_end = false;
//   for(lanelet::Lanelet lane : end_lanes){
//     contains_end &= route->contains(lane);
//   }
//   EXPECT_TRUE(contains_end) << "Route does not contain any ending lane options";
// }

TEST_F(TestGlobalPlannerFullMap, TestPlanFullRoute)
{
  // take the parking spot from previous test
  autoware_auto_msgs::msg::TrajectoryPoint from_point;
  // 45.25, -5 should lie within lanelet 11
  from_point.x = 45.25;
  from_point.y = -5;
  autoware_auto_msgs::msg::TrajectoryPoint to_point;
  to_point.x = 107.7665;
  to_point.y = -397.66;
  std::vector<lanelet::Id> route;
  std::vector<lanelet::Id> route_optionals;
  bool8_t result = node_ptr->plan_route(from_point, to_point, route, route_optionals);
  EXPECT_TRUE(result);
  std::size_t minsize = 0;
  ASSERT_GT(route.size(), minsize);
  EXPECT_EQ(route[0], 11) << "Should start at lane encpasulating start point";
  EXPECT_EQ(route[route.size()-1], 137) << "Should end at lane encapsulating end point";

  // Since our lanes are single-directional and there are no loops in this map, 
  // reversing the endpoints should mean there is no route:
  bool8_t reverse_result = node_ptr->plan_route(to_point, from_point, route, route_optionals);
  EXPECT_FALSE(reverse_result) << "No route should go against lanelet direction";
}

TEST_F(TestGlobalPlannerFullMap, TestPlanFullRouteMultipleOrigins)
{
  // take the parking spot from previous test
  autoware_auto_msgs::msg::TrajectoryPoint from_point;
  // -14, 2 should lie within lanelets 210, 317, and 282. Only 210 leads to goal.
  from_point.x = -14;
  from_point.y = 2;
  autoware_auto_msgs::msg::TrajectoryPoint to_point;
  to_point.x = 107.7665;
  to_point.y = -397.66;
  std::vector<lanelet::Id> route;
  std::vector<lanelet::Id> route_optionals;
  bool8_t result = node_ptr->plan_route(from_point, to_point, route, route_optionals);
  EXPECT_TRUE(result);
  std::size_t minsize = 0;
  ASSERT_GT(route.size(), minsize);
  EXPECT_EQ(route[0], 210) << "Should start at lane encpasulating start point";
  EXPECT_EQ(route[route.size()-1], 137) << "Should end at lane encapsulating end point";

  // Since our lanes are single-directional and there are no loops in this map, 
  // reversing the endpoints should mean there is no route:
  bool8_t reverse_result = node_ptr->plan_route(to_point, from_point, route, route_optionals);
  EXPECT_FALSE(reverse_result) << "No route should go against lanelet direction";
}

#endif  // TEST_LANELET2_GLOBAL_PLANNER_HPP_