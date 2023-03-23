/*
 * Package:   map_management
 * Filename:  RouteManager.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 *
 * Simple class to refine and update routes
 */

#pragma once

#include <chrono> // Time literals
#include <vector>
#include <string>

// Boost
#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

using namespace std::chrono_literals;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace navigator
{
    namespace planning
    {
        typedef bg::model::point<float, 2, bg::cs::cartesian> BoostPoint;
        typedef boost::geometry::model::linestring<BoostPoint> LineString;

        class RouteManager
        {
        public:
            // Constructor
            RouteManager();

            LineString getRoute(const LineString rough, const BoostPoint pos);
        };
    }
}