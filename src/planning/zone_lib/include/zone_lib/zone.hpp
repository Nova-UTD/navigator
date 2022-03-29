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

//libopendrive
#include "Mesh.h"
#include "Junction.h"
#include "OpenDriveMap.h"
#include "Road.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;
using PointMsg = geometry_msgs::msg::Point32;

namespace navigator
{
    namespace zones_lib
    {
        using ZoneMsg = voltron_msgs::msg::Zone;

        boost_polygon to_boost_polygon(const ZoneMsg& zone);
        boost_polygon to_boost_polygon(const odr::Mesh3D& mesh);
        
        ZoneMsg to_zone_msg(const boost_polygon& polygon);
        ZoneMsg to_zone_msg(const odr::Mesh3D& mesh);
        
        //need map to look up roads
        ZoneMsg to_zone_msg(odr::Junction& junction, odr::OpenDriveMap& map);

        // helper function to calculate point
        std::vector<PointMsg> calculate_corner_points(odr::LaneSection& lanesection, double s_val);


    } // namespace zones
    
} // namespace navigator