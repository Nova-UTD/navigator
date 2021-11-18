/*
 * Package:   path_planner
 * Filename:  parameterized_spline.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef PATH_PLANNER__PARAMETERIZED_SPLINE_HPP_
#define PATH_PLANNER__PARAMETERIZED_SPLINE_HPP_

#include <utility>
#include <path_planner/spline.hpp>

#pragma GCC diagnostic ignored "-Wsubobject-linkage" //disable this warning since the spline library uses an anonymous namespace

namespace navigator
{
    namespace path_planner
    {
        // represents a cubic spline:
        //   x(s) = a_x*s^3 + b_x*s^2 + c_x*s + d_x
        //   y(s) = a_y*s^3 + b_y*s^2 + c_y*s + d_y
        // we can sample a point at a given length along the curve s by evaluating (x(s),y(s))
        //      the arc length is based on the chord between points, so it's an approximation.
        //      using points close in distance helps

        class ParameterizedSpline
        {
        private:
            tk::spline sx, sy;

        public:
            ParameterizedSpline(tk::spline sx, tk::spline sy)
                : sx(sx), sy(sy) {}
            ParameterizedSpline(const std::vector<double> &x, const std::vector<double> &y);
            std::pair<double, double> sample(double s);
        };
    }
}

#endif
