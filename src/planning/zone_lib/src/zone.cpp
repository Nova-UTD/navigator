# include "zone_lib/zone.hpp"
#include "Road.h"

using PointMsg = geometry_msgs::msg::Point32;
using ZoneMsg = voltron_msgs::msg::Zone;


ZoneMsg navigator::zones_lib::to_zone_msg(const boost_polygon& polygon) {
    ZoneMsg output;
    output.max_speed = 0;
    output.cost = 0;
    for (auto poly_point : polygon.outer()) {
        PointMsg p;
        p.x = poly_point.x();
        p.y = poly_point.y();
        output.poly.points.push_back(p);
    }
    return output;
}

boost_polygon navigator::zones_lib::to_boost_polygon(const odr::Mesh3D& mesh){
    boost_polygon gon;

    std::vector<boost_point> points;
    // Split by odd/even since mesh alternates left, right, left, right, etc.
    for (int parity : {0, 1})
    {
        for (size_t i = parity; i < mesh.vertices.size(); i += 2)
        {
            auto vertex = mesh.vertices[i];
            boost::geometry::append(gon.outer(),boost_point(vertex[0], vertex[1]));
        }
    }
    boost::geometry::correct(gon);

    return gon;
}

ZoneMsg navigator::zones_lib::to_zone_msg(const odr::Mesh3D& mesh){
            return to_zone_msg(to_boost_polygon(mesh));
}

boost_polygon navigator::zones_lib::to_boost_polygon(const ZoneMsg& zone) {
    boost_polygon output;
    std::vector<boost_point> points;
    for (auto p : zone.poly.points) {
        boost::geometry::append(output.outer(), boost_point(p.x,p.y));
    }
    //just makes sure the points are in clockwise order
    boost::geometry::correct(output);
    return output;
}


std::vector<PointMsg> navigator::zones_lib::calculate_corner_points(odr::LaneSection& lanesection, double s_val) {

    std::shared_ptr<odr::Lane> leftmost_lane;
    std::shared_ptr<odr::Lane> rightmost_lane;
    int left_id = -100;
    int right_id = 100;

    for (auto const& [lane_id, lane] : lanesection.id_to_lane) {
        if (lane->type == "driving" && lane->id > left_id) {
            leftmost_lane = lane;
            left_id = lane->id;
        } else if (lane->type == "driving" && lane->id < right_id) {
            rightmost_lane = lane;
            right_id = lane->id;
        }
    }

    // get t vals
    double left_t = leftmost_lane->outer_border.get(s_val);
    double right_t = rightmost_lane->outer_border.get(s_val);
    
    // get x,y of each corner 
    odr::Vec3D left_pt = leftmost_lane->get_surface_pt(s_val, left_t);
    odr::Vec3D right_pt = rightmost_lane->get_surface_pt(s_val, right_t);

    PointMsg left_corner;
    left_corner.x = left_pt[0];
    left_corner.y = left_pt[1];

    PointMsg right_corner;
    right_corner.x = right_pt[0];
    right_corner.y = right_pt[1];

    std::vector<PointMsg> points{left_corner, right_corner};
    return points;
}


/*
    * for each incoming road into the junction (roads outside junction feeding in)
        * find s value of road that connects to intersection (either 0 or s_end)
        * find t values of outside farthest drivable lanes
        * sample lanesection to find real coordinates for that (s,t)
            * these are the two points touching the intersection on the far sides of the road
        * add points to msg_points

*/
ZoneMsg navigator::zones_lib::to_zone_msg(odr::Junction& junction, odr::OpenDriveMap& map) {
    
    std::vector<PointMsg> msg_points;

    for (auto const& [connection_id, junction_connection] : junction.connections) {

        // get connecting road and its lanesection (assuming only 1)
        std::shared_ptr<odr::Road> connecting_road = map.roads[junction_connection.connecting_road];
        std::shared_ptr<odr::LaneSection> lanesection = *(connecting_road->get_lanesections().begin());

        // get corners for start and end of lanesection
        std::vector<PointMsg> points_start = calculate_corner_points(*lanesection, 0);
        std::vector<PointMsg> points_end = calculate_corner_points(*lanesection, connecting_road->length);

        msg_points.push_back(points_start[0]);
        msg_points.push_back(points_start[1]);
        msg_points.push_back(points_end[0]);
        msg_points.push_back(points_end[1]);

    }

    ZoneMsg msg;
    msg.poly.points = msg_points;
    msg.max_speed = 0;
    msg.cost = 0;
    return msg;
}


