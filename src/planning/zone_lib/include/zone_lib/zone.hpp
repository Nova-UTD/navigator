/*
 * Package:   zone_lib
 * Filename:  zone.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */
#pragma once

#include "nova_msgs/msg/zone.hpp"

//libopendrive
#include "Mesh.h"
#include "Junction.h"
#include "OpenDriveMap.h"
#include "Road.h"

#include "opendrive_utils/OpenDriveUtils.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// #include "rclcpp/rclcpp.hpp"

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;
using PointMsg = geometry_msgs::msg::Point32;

namespace navigator
{
    namespace zones_lib
    {
        using ZoneMsg = nova_msgs::msg::Zone;
        using PointMsg = geometry_msgs::msg::Point32;

        boost_polygon to_boost_polygon(const ZoneMsg& zone);
        boost_polygon to_boost_polygon(const odr::Mesh3D& mesh);
        
        ZoneMsg to_zone_msg(const boost_polygon& polygon);
        ZoneMsg to_zone_msg(const odr::Mesh3D& mesh);
        
        //need map to look up roads
        ZoneMsg to_zone_msg(std::shared_ptr<odr::Junction> junction, navigator::opendrive::types::OpenDriveMapPtr map);

        // helper function to calculate point
        std::vector<PointMsg> calculate_corner_points(std::shared_ptr<odr::LaneSection> lanesection, double s_val);

        class ClockwiseComparator {
        public:
            double mean_x, mean_y;
            ClockwiseComparator(double mean_x, double mean_y) : mean_x(mean_x), mean_y(mean_y) {}
            bool operator()(const PointMsg& a, const PointMsg& b) const;
        };



    } // namespace zones
    
} // namespace navigator