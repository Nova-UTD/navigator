/*
 * Package:   path_planner
 * Filename:  test_path_planner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <path_planner/path_planner.hpp>
#include <geometry/common_2d.hpp>
#include <memory>

#include "gtest/gtest.h"

using autoware_auto_msgs::msg::MapPrimitive;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::HADMapSegment;
using autoware_auto_msgs::msg::TrajectoryPoint;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

// returns a map with a lane has given number of points(n_points)
// length of the lane will be n_points meters in y direction
lanelet::LaneletMapPtr getALaneletMapWithLaneId(
  const lanelet::Id & id, const float64_t velocity,
  const size_t n_points)
{
  lanelet::Points3d right_points, left_points, center_points;
  constexpr float64_t resolution = 1.0;
  for (size_t i = 0; i < n_points; i++) {
    const auto y = resolution * static_cast<float64_t>(i);
    left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), -1, y, 0));
    right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, y, 0));
    center_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, y, 0));
  }
  lanelet::LineString3d ls1(lanelet::utils::getId(), left_points);
  lanelet::LineString3d ls2(lanelet::utils::getId(), right_points);
  lanelet::LineString3d ls3(lanelet::utils::getId(), center_points);

  lanelet::Lanelet ll(id, ls1, ls2);
  ll.setCenterline(ls3);
  ll.setAttribute(lanelet::AttributeName::SpeedLimit, velocity);

  return lanelet::utils::createMap({ll});
}

HADMapRoute getARoute(const int64_t lane_id, const float32_t length)
{
  HADMapRoute had_map_route;
  had_map_route.start_point.position.x = 0;
  had_map_route.start_point.position.y = 0;
  had_map_route.goal_point.position.x = 0;
  had_map_route.goal_point.position.y = length;

  MapPrimitive primitive;
  primitive.id = lane_id;
  HADMapSegment segment;
  segment.preferred_primitive_id = primitive.id;
  had_map_route.segments.push_back(segment);
  had_map_route.segments.front().primitives.push_back(primitive);

  return had_map_route;
}

double sqr_distance(TrajectoryPoint a, TrajectoryPoint b) {
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}

class PathPlannerTest : public ::testing::Test
{
public:
  PathPlannerTest()
  {
    m_planner_ptr = std::make_shared<navigator::path_planner::PathPlanner>();
  }
  std::shared_ptr<navigator::path_planner::PathPlanner> m_planner_ptr;
};

TEST_F(PathPlannerTest, get_center_line)
{
  // create map
  const auto lane_id = lanelet::utils::getId();
  constexpr float64_t velocity_mps = 1.0;
  constexpr size_t n_points = 5;
  const auto lanelet_map_ptr = getALaneletMapWithLaneId(lane_id, velocity_mps, n_points);

  // create route message
  const auto had_map_route = getARoute(lane_id, 5.0F);

  const std::vector<TrajectoryPoint> points = m_planner_ptr->get_center_line_points(had_map_route, lanelet_map_ptr, 1);

  // return points should not be empty
  ASSERT_FALSE(points.empty());
  // there should be 5 points total
  ASSERT_EQ(size_t(5),points.size());

  //testing start point position
  //we expect the center line to be: (0,1) -> (0,2) -> (0,3) -> (0,4) -> (0,5)
  auto prev = points.front();
  ASSERT_DOUBLE_EQ(prev.x, 0);
  ASSERT_DOUBLE_EQ(prev.y, 1);
  //points should be spaced out by 1 vertically
  for (size_t i = 1; i < points.size(); i++) {
    ASSERT_DOUBLE_EQ(prev.y + 1, points[i].y);
    ASSERT_DOUBLE_EQ(prev.x, points[i].x);
    prev = points[i];
  }
}
