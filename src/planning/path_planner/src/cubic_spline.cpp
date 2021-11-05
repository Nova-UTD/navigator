// Copyright 2020 The Autoware Foundation
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

#include "path_planner/cubic_spline.hpp"

#include <math.h>

namespace navigator {
namespace path_planner {

std::pair<double, double> cubic_spline::sample(double arclength) {
    double s2 = arclength*arclength;
    double s3 = arclength*s2;
    double xs = ax*s3 + bx*s2 + cx*arclength + dx;
    double ys = ay*s3 + by*s2 + cy*arclength + dy;
    return std::make_pair(xs,ys);
}

std::pair<double, double> cubic_spline::sample_first_derivative(double arclength) {
    double s2 = arclength*arclength;
    double xs = 3*ax*s2 + 2*bx*arclength + cx;
    double ys = 3*ay*s2 + 2*by*arclength + cy;
    return std::make_pair(xs,ys);
}

std::pair<double, double> cubic_spline::sample_second_derivative(double arclength) {
    double xs = 6*ax*arclength + bx;
    double ys = 6*ay*arclength + by;
    return std::make_pair(xs,ys);
}

double cubic_spline::sample_heading(double arclength) {
    //equation from Hu paper
    auto derivative = sample_first_derivative(arclength);
    return atan2(derivative.second, derivative.first);
}

double cubic_spline::sample_curvature(double arclength) {
    //equation from Hu paper
    auto first_derivative = sample_first_derivative(arclength);
    auto second_derivative = sample_second_derivative(arclength);
    double denom = first_derivative.first + first_derivative.second;
    return (first_derivative.first*second_derivative.second - second_derivative.first*first_derivative.second)
        / sqrt(denom*denom*denom);
}

}
}