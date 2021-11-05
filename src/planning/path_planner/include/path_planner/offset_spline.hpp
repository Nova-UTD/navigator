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

#ifndef PATH_PLANNER__OFFSET_SPLINE_HPP_
#define PATH_PLANNER__OFFSET_SPLINE_HPP_

#include <iostream>
#include <vector>
#include <tuple>

namespace navigator
{
namespace path_planner
{
    //represents a curve with a lateral offset from a cubic spline:
    //  p(s) = a(s-start_arclength)^3 + b(s-start_arclength)^2 + c(s-start_arclength) + offset_start
    //this curve is in a different coordinate system than the cartesian plane, so we have to convert
    //  before using the coordinates in other places.

    class offset_spline
    {
    private:
        double start_arclength, end_arclength;
        double a,b,c,offset_start,offset_end;         //coefficients
        double da,db,dc;                              //coefficients for first derivative wrt s
        double d2a,d2b;                               //coefficients for second derivative wrt s

        void populate_derivatives();
    public:
        
        offset_spline(double start_arclength, double end_arclength, double offset_start, double offset_end, double a, double b, double c)
            : start_arclength(start_arclength), end_arclength(end_arclength), offset_start(offset_start), offset_end(offset_end), a(a), b(b), c(c) {
                populate_derivatives();
        }
        ~offset_spline() {}
        std::tuple<double, double> sample(double arclength);
        std::tuple<double, double> sample_offset(double arclength, double lateral_offset);
        std::tuple<double, double> sample_first_derivative(double arclength);
        std::tuple<double, double> sample_second_derivative(double arclength);
        double sample_curvature(double arclength);
        double sample_heading(double arclength);
        double sample_heading_offset(double arclength, double lateral_offset);

    };
}
}



#endif  // PATH_PLANNER__OFFSET_SPLINE_HPP_
