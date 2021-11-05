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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lane_planner class.

#ifndef PATH_PLANNER__CUBIC_SPLINE_HPP_
#define PATH_PLANNER__CUBIC_SPLINE_HPP_

#include <utility>

namespace navigator
{
namespace path_planner
{
    //represents a cubic spline:
    //  x(s) = a_x*s^3 + b_x*s^2 + c_x*s + d_x
    //  y(s) = a_y*s^3 + b_y*s^2 + c_y*s + d_y
    //we can sample a point at a given length along the curve s by evaluating (x(s),y(s))

    class cubic_spline
    {
    private:
        double ax,bx,cx,dx,ay,by,cy,dy;         //coefficients for x(s) and y(s)

    public:
        
        cubic_spline(double start_arclength, double end_arclength, double ax, double bx, double cx, double dx, double ay, double by, double cy, double dy)
            : ax(ax), bx(bx), cx(cx), dx(dx), ay(ay), by(by), cy(cy), dy(dy) {
        }
        ~cubic_spline() {}
        std::pair<double, double> sample(double arclength);
        std::pair<double, double> sample_first_derivative(double arclength);
        std::pair<double, double> sample_second_derivative(double arclength);
        double sample_curvature(double arclength);
        double sample_heading(double arclength);

    };
}
}

#endif  // PATH_PLANNER__CUBIC_SPLINE_HPP_
