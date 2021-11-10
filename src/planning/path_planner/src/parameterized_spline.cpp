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

#include "path_planner/parameterized_spline.hpp"

#include <math.h>

namespace navigator
{
    namespace path_planner
    {
        std::pair<double, double> ParameterizedSpline::sample(double s)
        {
            return std::pair(sx(s), sy(s));
        }
        ParameterizedSpline::ParameterizedSpline(const std::vector<lanelet::ConstPoint3d> &points)
        {
            std::vector<double> T(0, points.size());

            // time proportional to distance, i.e. traverse the curve at constant speed
            // approximate arclength parameter by using straight line distance between points
            T[0] = 0;
            for (size_t i = 1; i < T.size(); i++)
                T[i] = T[i - 1] + sqrt((points[i].x() - points[i - 1].x()) * (points[i].x() - points[i - 1].x()) + (points[i].y() - points[i - 1].y()) * (points[i].y() - points[i - 1].y()));
            // setup splines for x and y coordinate
            sx = tk::spline(T, X);
            sy = tk::spline(T, Y);
        }
    }
}