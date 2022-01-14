/*
 * Package:   path_planner
 * Filename:  test_segemented_path.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Basic tests for the segmented_path

#include <gtest/gtest.h>
#include <memory>
#include <math.h>

#include <path_planner/segmented_path.hpp>
#include <path_planner/path_point.hpp>

using namespace navigator::path_planner;

class test_segmented_path : public ::testing::Test {
public:
  std::shared_ptr<std::vector<path_point>> ps;
  
  test_segmented_path() {
    ps = std::make_shared<std::vector<path_point>>();
    //wiggly path that goes loops around and is not a function of x or y
    ps->push_back(path_point(0,0));
    ps->push_back(path_point(0.1,0.1));
    ps->push_back(path_point(0,0.2));
    ps->push_back(path_point(-0.1,0.1));
    ps->push_back(path_point(-0.2,0));
    ps->push_back(path_point(-0.3,-0.1));

    segments = std::make_shared<segmented_path>(ps);
  };
  std::shared_ptr<segmented_path> segments;
};

TEST_F(test_segmented_path, test_valid) {
  ASSERT_TRUE(segments->valid_points());
}

TEST_F(test_segmented_path, test_arc_length_parameterization) {
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

TEST_F(test_segmented_path, test_closest_point_distance) {
  const double tolerance = 0.000001;
  auto p = path_point(0,0);
  ASSERT_EQ(0, segments->distance(p));
  p = path_point(-0.3,-0.1);
  ASSERT_EQ(0, segments->distance(p));
  p = path_point(0,0.2);
  ASSERT_EQ(0, segments->distance(p));
  p = path_point(0.2,0.2);
  ASSERT_TRUE(abs(std::sqrt(0.02)-segments->distance(p)) < tolerance);
}

TEST_F(test_segmented_path, test_branch) {
  using std::size_t;
  //should create a branch at 0,0 initially going in the positive x direction, that increases the steering angle until it
  //  eventually reaches PI/2 radians per point
  //the turn speed is 1 radian/unit traveled
  //spacing is sqrt(2) like the other path
  auto points = *segments->create_branch(0, M_PI_2, 1, 20).get();
  ASSERT_TRUE(segments->valid_points(segments->spacing, points));
  ASSERT_EQ(20, points.size());
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