/*
 * Package:   opendrive_utils
 * Filename:  test_OpenDriveUtils.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "opendrive_utils/OpenDriveUtils.hpp"
#include "opendrive_utils/GeometricMap.hpp"

using namespace navigator::opendrive::types;
using namespace navigator::opendrive;

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include) && \
    __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <common/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif

class OpenDriveUtilsBaseTest : public ::testing::Test
{
public:
  OpenDriveMapPtr map;

  OpenDriveUtilsBaseTest(
      std::string map_file_path)
  {
    std::string root_folder =
        ament_index_cpp::get_package_share_directory("opendrive_utils");

    map_file_path = root_folder + map_file_path;
    map = load_map(map_file_path);
  }
};

class OpenDriveUtilsSimpleTest : public OpenDriveUtilsBaseTest
{
public:
  OpenDriveUtilsSimpleTest()
      : OpenDriveUtilsBaseTest("/test/maps/simple_map.xodr")
  {
  }
};

TEST_F(OpenDriveUtilsSimpleTest, test_load_map)
{
  ASSERT_EQ(map->get_roads().size(), 84ul) << "Map is not empty and has correct number of roads.";
}

TEST_F(OpenDriveUtilsSimpleTest, test_graph_lane_indentifiers)
{
  // Make sure that if we build a LaneIdentifier in the correct way,
  // the lane we get back is the same as the lane we put in.
  LaneGraph lane_graph(map);

  for (RoadPtr r : map->get_roads())
  {
    RoadId r_id = r->id;
    for (LaneSectionPtr ls : r->get_lanesections())
    {
      LaneSectionId ls_id = ls->s0;
      for (LanePtr l : ls->get_lanes())
      {
        LaneId l_id = l->id;
        LaneIdentifier lane_id = {r_id, ls_id, l_id};
        EXPECT_EQ(lane_graph.get_lane(lane_id), l) << "LaneGraph did not return the correct lane.";
      }
    }
  }
}

//TEST_F(OpenDriveUtilsSimpleTest, test_point_to_lane){
//  LaneGraph lane_graph(map);
//
//  LaneIdentifier test_lane_id = {"12",0.0, 1};
//  double test_x = 96.67;
//  double test_y = -109.31;
//
//  LanePtr test_lane = lane_graph.get_lane(test_lane_id);
//  
//  EXPECT_TRUE(contains(test_lane, test_x, test_y, map->x_offs, map->y_offs));
//
//  GeometricMap geometric_map(map, 50);
//  std::vector<LanePtr> lanes;
//  try{
//  EXPECT_TRUE(geometric_map.containing_lanes(test_x, test_y, lanes)) << "Lanes containing point were not found.";
//  } catch(std::exception& e){
//    std::cout << "Exception: " << e.what() << std::endl;
//  }
//  ASSERT_EQ(lanes.size(), 1ul) << "Only one lane should contain test point.";
//  EXPECT_EQ(lanes[0], test_lane) << "Lane containing test point is not correct.";
//}