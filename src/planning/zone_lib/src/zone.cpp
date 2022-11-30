#include "zone_lib/zone.hpp"
#include <math.h>
#include <algorithm>

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


std::vector<PointMsg> navigator::zones_lib::calculate_corner_points(std::shared_ptr<odr::LaneSection> lanesection, double s_val) {

    std::shared_ptr<odr::Lane> leftmost_lane;
    std::shared_ptr<odr::Lane> rightmost_lane;
    int left_id = -100;
    int right_id = 100;
    for (auto const& [lane_id, lane] : lanesection->id_to_lane) {
        if (lane->type == "driving" && lane->id > left_id) {
            leftmost_lane = lane;
            left_id = lane->id;
        } 
        if (lane->type == "driving" && lane->id < right_id) {
            rightmost_lane = lane;
            right_id = lane->id;
        }
    }
    std::vector<PointMsg> points;
    // get t vals, then x,y
    //not sure why these are null sometimes...
    if (leftmost_lane != nullptr) {
        double left_t = leftmost_lane->outer_border.get(s_val);
        odr::Vec3D left_pt = leftmost_lane->get_surface_pt(s_val, left_t);
        PointMsg left_corner;
        left_corner.x = left_pt[0];
        left_corner.y = left_pt[1];
        points.push_back(left_corner);
    }
    if (rightmost_lane != nullptr) {
        double right_t = rightmost_lane->outer_border.get(s_val);
        odr::Vec3D right_pt = rightmost_lane->get_surface_pt(s_val, right_t);
        PointMsg right_corner;
        right_corner.x = right_pt[0];
        right_corner.y = right_pt[1];
        points.push_back(right_corner);
    }
    
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
ZoneMsg navigator::zones_lib::to_zone_msg(std::shared_ptr<odr::Junction> junction, navigator::opendrive::types::OpenDriveMapPtr map) {
    
    std::vector<PointMsg> msg_points;
    for (auto const& [connection_id, junction_connection] : junction->connections) {

        // get connecting road and its lanesection (assuming only 1)
        std::shared_ptr<odr::Road> connecting_road = map->roads[junction_connection.connecting_road];
        std::shared_ptr<odr::LaneSection> lanesection = *(connecting_road->get_lanesections().begin());

        // get corners for start and end of lanesection
        std::vector<PointMsg> points_start = calculate_corner_points(lanesection, 0);
        std::vector<PointMsg> points_end = calculate_corner_points(lanesection, connecting_road->length);
        for (auto pt : points_start) {
            msg_points.push_back(pt);
        }
        for (auto pt : points_end) {
            msg_points.push_back(pt);
        }
    }

    // reorient point, sort by angle clockwise about the centroid
    double mean_x = 0, mean_y = 0;
    for (const PointMsg& pt : msg_points) {
        mean_x += pt.x;
        mean_y += pt.y;
    }
    ClockwiseComparator cmp(mean_x/msg_points.size(), mean_y/msg_points.size());
    std::sort(msg_points.begin(), msg_points.end(), cmp);
    
    ZoneMsg msg;
    msg.poly.points = msg_points;
    msg.max_speed = 0;
    msg.cost = 0;
    return msg;
}

bool navigator::zones_lib::ClockwiseComparator::operator()(const PointMsg& a, const PointMsg& b) const {
    double a_x = a.x-this->mean_x;
    double a_y = a.y-this->mean_y;
    double b_x = b.x-this->mean_x;
    double b_y = b.y-this->mean_y;
    double angle_a = std::atan2(a_y,a_x);
    double angle_b = std::atan2(b_y,b_x);
    //special case between quadrants 2 and 3
    //between them, the angle flips from +pi to -pi.
    if (a_x <= 0 && a_y >= 0 && b_x <= 0 && b_y <= 0) {
        //a in quadrant 2, b in quadrant 3, a comes before b
        return true;
    }
    if (b_x <= 0 && b_y >= 0 && a_x <= 0 && a_y <= 0) {
        //a in quadrant 3, b in quadrant 2, b comes before a
        return false;
    }
    //otherwise, radians decrease clockwise
    return angle_a > angle_b;
}
