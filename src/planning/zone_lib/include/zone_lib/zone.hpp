/*
 * Package:   zone_lib
 * Filename:  zone.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */
#pragma once

#include "voltron_msgs/msg/zone.hpp"
#include "Mesh.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

namespace navigator
{
    namespace zones_lib
    {
        using ZoneMsg = voltron_msgs::msg::Zone;

        boost_polygon to_boost_polygon(const ZoneMsg& zone);
        boost_polygon to_boost_polygon(odr::Mesh3D mesh);
        
        ZoneMsg to_zone_msg(const boost_polygon& polygon);
        ZoneMsg to_zone_msg(const odr::Mesh3D& mesh){
            return to_zone_msg(to_boost_polygon(mesh));
        }

    } // namespace zones
    
} // namespace navigator