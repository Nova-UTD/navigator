/*
 * Package:   path_planner
 * Filename:  test_parameterized_spline.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Basic tests for the splines

#include <gtest/gtest.h>
#include <memory>

#include <path_planner/parameterized_spline.hpp>

using namespace navigator::path_planner;

class TestSpline : public ::testing::Test {
public:

  TestSpline() : spline(std::vector<double>(),std::vector<double>()) {
    xs.push_back(0);
    ys.push_back(0);

    xs.push_back(0.1);
    ys.push_back(0.2);

    xs.push_back(0.5);
    ys.push_back(0.5);

    xs.push_back(0.2);
    ys.push_back(0.7);

    spline = ParameterizedSpline(xs, ys);
  };
  ParameterizedSpline spline;
  std::vector<double> xs;
  std::vector<double> ys;
};


TEST_F(TestSpline, test_arc_length_parameterization) {
    //estimate arc length by evaluating at small intervals
    double arclength = 0;
    double step_size = 0.1;
    auto old_point = spline.sample(0);
    for (double p = 0; p < 2; p += step_size) {
      auto current_point = spline.sample(p);
      arclength += sqrt((old_point.first - current_point.first) * (old_point.first - current_point.first) + (old_point.second - current_point.second) * (old_point.second - current_point.second));
      old_point = current_point;
    }
    //last point sampled should be approx equal to point sampled at estimated arc length 
    ASSERT_TRUE(abs(old_point.first-spline.sample(arclength).first < 0.1));
    ASSERT_TRUE(abs(old_point.second-spline.sample(arclength).second < 0.1));
}