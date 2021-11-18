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
  std::vector<double> xs;
  std::vector<double> ys;
  
  TestSpline() {
    //wiggly path that goes loops around and is not a function of x or y
    xs.push_back(0);
    ys.push_back(0);

    xs.push_back(0.1);
    ys.push_back(0.1);

    xs.push_back(0.2);
    ys.push_back(0.1);

    xs.push_back(0.3);
    ys.push_back(0.2);

    xs.push_back(0.3);
    ys.push_back(0.3);

    xs.push_back(0.2);
    ys.push_back(0.3);

    xs.push_back(0.2);
    ys.push_back(0.2);

    xs.push_back(0.1);
    ys.push_back(0.3);

    xs.push_back(0);
    ys.push_back(0.2);

    spline = std::make_shared<ParameterizedSpline>(xs, ys);
  };
  std::shared_ptr<ParameterizedSpline> spline;
};


TEST_F(TestSpline, test_arc_length_parameterization) {
    const double tolerance = 0.04;
    const double length_step = 0.1;
    //validate arc length parameterization by seeing if the distance over small intervals is equal to the change in the parameter
    auto old = spline->sample(0);
    for (double s = length_step; s < 1; s += length_step) {
      auto current = spline->sample(s);
      double d = sqrt((old.first-current.first)*(old.first-current.first) + (old.second-current.second)*(old.second-current.second));
      old = current;
      ASSERT_TRUE(abs(d-length_step) < tolerance);
    }
}