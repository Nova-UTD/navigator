/*
 * Package:   MotionPlanner
 * Filename:  test_segemented_path.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Basic tests for the SegmentedPath

#include <gtest/gtest.h>
#include <memory>
#include <math.h>

#include <motion_planner/SegmentedPath.hpp>
#include <motion_planner/PathPoint.hpp>

using namespace navigator::MotionPlanner;

class test_SegmentedPath : public ::testing::Test {
public:
  std::shared_ptr<std::vector<PathPoint>> ps;
  
  test_SegmentedPath() {
    ps = std::make_shared<std::vector<PathPoint>>();
    //wiggly path that goes loops around and is not a function of x or y
    ps->push_back(PathPoint(0,0));
    ps->push_back(PathPoint(0.1,0.1));
    ps->push_back(PathPoint(0,0.2));
    ps->push_back(PathPoint(-0.1,0.1));
    ps->push_back(PathPoint(-0.2,0));
    ps->push_back(PathPoint(-0.3,-0.1));

    segments = std::make_shared<SegmentedPath>(ps);
  };
  std::shared_ptr<SegmentedPath> segments;
};

TEST_F(test_SegmentedPath, test_valid) {
  ASSERT_TRUE(segments->valid_points());
}

TEST_F(test_SegmentedPath, test_arc_length_parameterization) {
    const double tolerance = 0.000001;
    const double length_step = std::sqrt(2);
    //validate arc length parameterization by seeing if the distance over small intervals is equal to the change in the parameter
    auto old = segments->sample(0);
    for (double s = length_step; s < 1; s += length_step) {
      auto current = segments->sample(s);
      double d = current.distance(old);
      old = current;
      ASSERT_TRUE(abs(d-length_step) < tolerance);
    }
}

TEST_F(test_SegmentedPath, test_closest_point_distance) {
  const double tolerance = 0.000001;
  auto p = PathPoint(0,0);
  ASSERT_EQ(0, segments->distance(p));
  p = PathPoint(-0.3,-0.1);
  ASSERT_EQ(0, segments->distance(p));
  p = PathPoint(0,0.2);
  ASSERT_EQ(0, segments->distance(p));
  p = PathPoint(0.2,0.2);
  ASSERT_TRUE(abs(std::sqrt(0.02)-segments->distance(p)) < tolerance);
}

TEST_F(test_SegmentedPath, test_branch) {
  using std::size_t;
  //should create a branch at 0,0 initially going in the positive x direction, that increases the steering angle until it
  //  eventually reaches PI/2 radians per point
  //the turn speed is 1 radian/unit traveled
  //spacing is sqrt(2) like the other path
  auto points = *segments->create_branch(0, M_PI_2, 1, 20).get();
  ASSERT_TRUE(segments->valid_points(segments->spacing, points));
  ASSERT_EQ(static_cast<size_t>(20), points.size());
  const double turn_per_segment = segments->spacing;
  const double tolerance = 0.0001;
  //round down because the last segment that turns doesn't turn all the way
  const size_t turns = (size_t)(M_PI_2 / turn_per_segment);

  //other points should turn the wheel at the desired speed until we reach the final angle
  for (size_t i = 0; i < turns-1; i++) {
    double turn_angle = std::atan2(points[i+1].y-points[i].y, points[i+1].x-points[i].x);
    ASSERT_TRUE(abs(turn_angle - turn_per_segment*(i+1)) < tolerance);
  }
  for (size_t i = turns; i < points.size()-1; i++) {
    //should be turning PI/2 each time now
    double turn_angle = std::atan2(points[i+1].y-points[i].y, points[i+1].x-points[i].x);
    ASSERT_TRUE(abs(turn_angle - M_PI_2) < tolerance);
  }
}

TEST_F(test_SegmentedPath, test_intersection) {
  using std::vector;
  //use a horizontal path that loops back on iteself and a vertical line
  auto points = std::make_shared<vector<PathPoint>>();
  points->push_back(PathPoint(0,0));
  points->push_back(PathPoint(0.5,0));
  points->push_back(PathPoint(1,0));
  points->push_back(PathPoint(1.5,0));
  points->push_back(PathPoint(1.5,0.5));
  points->push_back(PathPoint(1,0.5));
  points->push_back(PathPoint(0.5,0.5));
  SegmentedPath path(points, 0.5);
  //intersect with vertical line with point (0.6,-1)
  auto intersection = path.intersection(0,1,0.6,-1);
  ASSERT_EQ(2ul, intersection.size());
  ASSERT_EQ(0.6, intersection[0]);
}

TEST_F(test_SegmentedPath, test_intersection_x_slope) {
  using std::vector;
  //use a vertical path with horizontal line
  auto points = std::make_shared<vector<PathPoint>>();
  points->push_back(PathPoint(0,0));
  points->push_back(PathPoint(0,0.5));
  points->push_back(PathPoint(0,1));
  points->push_back(PathPoint(0,1.5));
  SegmentedPath path(points, 0.5);
  //intersect with horizontal line with point (-1,0.6)
  auto intersection = path.intersection(1,0,-1,0.6);
  ASSERT_EQ(1ul, intersection.size());
  ASSERT_EQ(0.6, intersection[0]);
}

TEST_F(test_SegmentedPath, test_intersection_weird_path) {
  using std::vector;
  //use the weird path and with a non-axis-aligned slope
  auto intersection = segments->intersection(-0.001,100,0.05,0);
  ASSERT_EQ(1ul, intersection.size());
  //make sure the intersections are on the right parts of each segment
  //not using exact values because of the weird slope 
  ASSERT_LT(0.07, intersection[0]);
  ASSERT_GT(0.08, intersection[0]);
}

TEST_F(test_SegmentedPath, test_heading) {
  using std::vector;
  auto points = std::make_shared<vector<PathPoint>>();
  points->push_back(PathPoint(0,0));
  points->push_back(PathPoint(0.5,0.5));
  points->push_back(PathPoint(0,1));
  points->push_back(PathPoint(-0.5,0.5));
  points->push_back(PathPoint(0,0));
  SegmentedPath path(points);
  ASSERT_EQ(M_PI_4, path.heading(0));
  ASSERT_EQ(3*M_PI_4, path.heading(1));
  ASSERT_EQ(-3*M_PI_4, path.heading(2));
  ASSERT_EQ(-M_PI_4, path.heading(3));

  points = std::make_shared<vector<PathPoint>>();
  points->push_back(PathPoint(0,0));
  points->push_back(PathPoint(1,0));
  points->push_back(PathPoint(1,1));
  points->push_back(PathPoint(0,1));
  points->push_back(PathPoint(0,0));
  SegmentedPath path2(points);
  ASSERT_EQ(0, path2.heading(0));
  ASSERT_EQ(M_PI_2, path2.heading(1));
  ASSERT_EQ(M_PI, path2.heading(2));
  ASSERT_EQ(-M_PI_2, path2.heading(3));
}