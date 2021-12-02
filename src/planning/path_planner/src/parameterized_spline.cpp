/*
 * Package:   path_planner
 * Filename:  parameterized_spline.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#include "path_planner/parameterized_spline.hpp"

#include <math.h>

namespace navigator
{
    namespace path_planner
    {
        std::pair<double, double> ParameterizedSpline::sample(double s)
        {
            return std::pair<double,double>(sx(s), sy(s));
        }
        ParameterizedSpline::ParameterizedSpline(const std::vector<double> &x, const std::vector<double> &y)
        {
            assert(x.size() == y.size());
            std::vector<double> T(x.size(), 0);

            // time proportional to distance, i.e. traverse the curve at constant speed
            // approximate arclength parameter by using straight line distance between points
            T[0] = 0;
            for (size_t i = 1; i < T.size(); i++)
                T[i] = T[i - 1] + sqrt((x[i] - x[i - 1]) * (x[i] - x[i - 1]) + (y[i] - y[i - 1]) * (y[i] - y[i - 1]));
            // setup splines for x and y coordinate
            sx = tk::spline(T, x);
            sy = tk::spline(T, y);
        }
    }
}