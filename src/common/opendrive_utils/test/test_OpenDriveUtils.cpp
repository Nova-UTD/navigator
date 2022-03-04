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

using navigator::opendrive::OpenDriveMapPtr;


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

  OpenDriveMapPtr map_ptr;

  OpenDriveUtilsBaseTest(
      std::string map_file_path)
  {
      std::string root_folder =
          ament_index_cpp::get_package_share_directory("opendrive_utils");

      map_file_path = root_folder + map_file_path;
      map_ptr = navigator::opendrive::load_map(map_file_path);
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
  ASSERT_EQ(map_ptr->get_roads().size(), 84) << "Map is not empty and has correct number of roads.";
}