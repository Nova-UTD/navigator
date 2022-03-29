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
        std::shared_ptr<odr::Road> incoming_road = map.roads[junction_connection.incoming_road];

        // assume by default that predecessor is junction and s-value is 0
        double s_val = 0.0;

        // successor is junction, s is length of road
        if (incoming_road->successor.type == RoadLink::Type::Junction && incoming_road->successor.id == junction.id) {
            s_val = incoming_road->length;
        }
        
        // since only 1 lanesection in XODR map, all lanes in 1 section
        std::shared_ptr<odr::LaneSection> lanesection = *(road->get_lanesections().begin());
        
        // lanesection map may not be in order, so find driving lanes with highest/lowest IDs
        std::shared_ptr<odr::Lane> leftmost_lane;
        std::shared_ptr<odr::Lane> rightmost_lane;
        int left_id = -100;
        int right_id = 100;

        for (auto const& [lane_id, lane] : lanesection->id_to_lane) {
            if (lane.type == "driving" && lane.id > left_id) {
                leftmost_lane = *(lane);
                left_id = lane.id;
            } else if (lane.type == "driving" && lane.id < right_id) {
                rightmost_lane = *(lane);
                right_id = lane.id;
            }
        }

        // get t vals
        double left_t = leftmost_lane.outer_border.get(s_val);
        double right_t = rightmost_lane.outer_border.get(s_val);
        
        // get x,y of each corner 
        Vec3D left_pt = leftmost_lane.get_surface_pt(s, left_t);
        Vec3D right_pt = rightmost_lane.get_surface_pt(s, right_t);

        PointMsg left_corner;
        left_corner.x = left_pt.x;
        left_corner.y = left_pt.y;
        msg_points.push_back(left_corner);

        PointMsg right_corner;
        right_corner.x = right_pt.x;
        right_corner.y = right_pt.y;
        msg_points.push_back(right_corner);
        
        // reorient point ???

    }

    ZoneMsg msg;
    msg.poly.points = msg_points;
    msg.max_speed = 0;
    msg.cost = 0;
    return msg;
}