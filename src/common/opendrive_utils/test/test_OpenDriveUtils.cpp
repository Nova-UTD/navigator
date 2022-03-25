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
