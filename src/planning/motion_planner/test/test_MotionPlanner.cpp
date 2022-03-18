/*
 * Package:   MotionPlanner
 * Filename:  test_MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <motion_planner/MotionPlanner.hpp>
#include <geometry/common_2d.hpp>
#include <memory>
#include "voltron_msgs/msg/final_path.hpp"

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

//returns a HAD map route with one segment that starts at (0,0) and goes to (0,length)
//this lets us test the motion planner on a very simple route with easily predictable output
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

geometry_msgs::msg::Point get_ros_point(double x, double y)
{
    geometry_msgs::msg::Point p;
    p.set__x(x);
    p.set__y(y);
    return p;
}

class MotionPlannerTest : public ::testing::Test
{
public:
  MotionPlannerTest()
  {
    m_planner_ptr = std::make_shared<navigator::MotionPlanner::MotionPlanner>();
  }
  std::shared_ptr<navigator::MotionPlanner::MotionPlanner> m_planner_ptr;
};

TEST_F(MotionPlannerTest, get_center_line)
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

TEST_F(MotionPlannerTest, test_collision) {
  using namespace navigator::MotionPlanner;
  //create vertical path from (0,0) to (0,4)
  auto points = std::make_shared<std::vector<PathPoint>>();
  points->push_back(PathPoint(0,0));
  points->push_back(PathPoint(0,1));
  points->push_back(PathPoint(0,2));
  points->push_back(PathPoint(0,3));
  points->push_back(PathPoint(0,4));
  auto path = SegmentedPath(points, 1);
  ASSERT_TRUE(path.valid_points());

  //create objects to collide with
  std::vector<CarPose> objects;
  //starts at (-5,2) with velocity (1,0).steering angle=0. bounding box is size (2,1)
  objects.push_back(CarPose(-5,2,0,1,0,2,1,0));
  //starts at (3,1) with velocity (1,0). steering angle=0. bounding box is size (0.5,0.5)
  //this one should never hit the path
  objects.push_back(CarPose(3,1,0,1,0,0.5,0.5,0));

  auto collisions = m_planner_ptr->get_collisions(path, objects);

  //this should yield 2 collisions (one for the left and right sides of the car)
  ASSERT_EQ(2ul, collisions.size());
  //should keep y coordinate for each collision (y coordinate = arclength here)
  ASSERT_EQ(2.5, collisions[0].s_in);
  ASSERT_EQ(1.5, collisions[1].s_in);
  //they should be for the first car. left comes before right
  //time until collision is 5 seconds
  ASSERT_EQ(4, collisions[0].t_in);
  ASSERT_EQ(4, collisions[1].t_in);
  //car will be on the path for 2 seconds
  ASSERT_EQ(6, collisions[0].t_out);
  ASSERT_EQ(6, collisions[1].t_out);

  //test safety padding
  //safe entry should be earlier than actual by some constant
  ASSERT_EQ(4-m_planner_ptr->following_time, collisions[0].t_safe_in);
  ASSERT_EQ(4-m_planner_ptr->following_time, collisions[1].t_safe_in);
  //safe exit should be later than actual by some constant
  ASSERT_EQ(6+m_planner_ptr->following_time, collisions[0].t_safe_out);
  ASSERT_EQ(6+m_planner_ptr->following_time, collisions[1].t_safe_out);
  //safe entry arclength should be before the actual by some constant
  ASSERT_EQ(2.5-m_planner_ptr->following_distance, collisions[0].s_safe_in);
  ASSERT_EQ(1.5-m_planner_ptr->following_distance, collisions[1].s_safe_in);
}

TEST_F(MotionPlannerTest, test_linear_speed) {
  using namespace navigator::MotionPlanner;
  //create vertical path from (0,0) to (0,4)
  auto path = std::make_shared<voltron_msgs::msg::FinalPath>();
  double speed_limit = 8.94; //can be any number
  for (size_t i = 0; i < 100; i++)
  {
    path->points.push_back(get_ros_point(0,i));
    path->speeds.push_back(speed_limit);
  }

  std::shared_ptr<std::vector<SegmentedPath>> paths = m_planner_ptr->get_trajectory(path, CarPose(0,0,M_PI_2,0,speed_limit), std::vector<CarPose>());
  //linear path is first
  SegmentedPath linear_path = paths->at(0);
  for (size_t i = 0; i < linear_path.points->size(); i++) {
    //should be full speed up
    ASSERT_EQ(speed_limit, abs(linear_path.points->at(i).vy));
    ASSERT_LT(abs(linear_path.points->at(i).vx), 0.0000001); //effectively 0
  }

}

TEST_F(MotionPlannerTest, test_accel_smoothing) {
  using namespace navigator::MotionPlanner;
  //create vertical path from (0,0) to (0,3)
  auto path = std::make_shared<voltron_msgs::msg::FinalPath>();
  double speed_limit = 9999; //big speed limit will never be reached
  for (size_t i = 0; i < 100; i++)
  {
    path->points.push_back(get_ros_point(0,i));
    path->speeds.push_back(speed_limit);
  }

  
  double a = m_planner_ptr->max_accel;
  double d = m_planner_ptr->spacing;
  //distance until we reach the speed limit. d=v^2/(2a)
  size_t d_to_reach = std::floor(speed_limit*speed_limit/(2*a)/d);

  std::shared_ptr<std::vector<SegmentedPath>> paths = m_planner_ptr->get_trajectory(path, CarPose(0,0,M_PI_2,0,0), std::vector<CarPose>());
  //linear path is first
  SegmentedPath linear_path = paths->at(0);

  PathPoint p = linear_path.points->at(0);
  double ke = p.vx*p.vx+p.vy*p.vy;
  ASSERT_EQ(0, ke); //car beings at rest
  
  for (size_t i = 1; i < linear_path.points->size() && i < d_to_reach; i++) {
    //should be full speed up
    PathPoint p = linear_path.points->at(i);
    double speed_sqr = p.vx*p.vx+p.vy*p.vy;
    double delta_ke = 0.5*speed_sqr - ke;
    ke = 0.5*speed_sqr;
    ASSERT_FLOAT_EQ(m_planner_ptr->max_accel*d, delta_ke);
  }

  for (size_t i = d_to_reach+1; i < linear_path.points->size(); i++) {

    //we have accelerated to max speed, so all points should be max speed
    //not guaranteed to be tested
    PathPoint p = linear_path.points->at(i);
    double speed_sqr = p.vx*p.vx+p.vy*p.vy;
    ASSERT_FLOAT_EQ(speed_limit*speed_limit, speed_sqr);
  }

}

/*TEST_F(MotionPlannerTest, test_brake) {
  using namespace navigator::MotionPlanner;
  //create vertical path from (0,0) to (0,3)
  auto path = std::make_shared<voltron_msgs::msg::FinalPath>();
  double speed_limit = 9999; //big speed limit will never be reached
  double trajectory_length = m_planner_ptr->spacing*m_planner_ptr->points;
  size_t boundary = static_cast<size_t>(std::floor(trajectory_length)-1);
  for (size_t i = 0; i < boundary; i++)
  {
    path->points.push_back(get_ros_point(0,i));
    path->speeds.push_back(speed_limit);
  }
  for (size_t i = boundary; i < 100; i++)
  {
    path->points.push_back(get_ros_point(0,i));
    path->speeds.push_back(0);
  }

  
  double a = m_planner_ptr->max_accel;
  double d = m_planner_ptr->spacing;
  double b = m_planner_ptr->max_brake_accel;
  //distance until we reach the speed limit. d=v^2/(2a)
  size_t d_to_reach = std::floor(speed_limit*speed_limit/(2*a)/d);

  std::shared_ptr<std::vector<SegmentedPath>> paths = m_planner_ptr->get_trajectory(path, CarPose(0,0,M_PI_2,0,0), std::vector<CarPose>());
  //linear path is first
  SegmentedPath linear_path = paths->at(0);

  PathPoint p = linear_path.points->at(0);
  double ke = p.vx*p.vx+p.vy*p.vy;
  ASSERT_EQ(0, ke); //car beings at rest
  
  for (size_t i = 1; i < linear_path.points->size() && i < d_to_reach; i++) {
    //should be full speed up
    PathPoint p = linear_path.points->at(i);
    double speed_sqr = p.vx*p.vx+p.vy*p.vy;
    double delta_ke = 0.5*speed_sqr - ke;
    ke = 0.5*speed_sqr;
    ASSERT_FLOAT_EQ(m_planner_ptr->max_accel*d, delta_ke);
  }

  for (size_t i = d_to_reach+1; i < linear_path.points->size(); i++) {

    //we have accelerated to max speed, so all points should be max speed
    //not guaranteed to be tested
    PathPoint p = linear_path.points->at(i);
    double speed_sqr = p.vx*p.vx+p.vy*p.vy;
    ASSERT_FLOAT_EQ(speed_limit*speed_limit, speed_sqr);
  }

}*/