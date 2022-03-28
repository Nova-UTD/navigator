# include "zone_lib/zone.hpp"

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
ZoneMsg navigator::zones_lib::to_zone_msg(const odr::Junction& junction, const odr::OpenDriveMap& map) {
    std::vector<PointMsg> msg_points;
    for (JunctionConnection jc : junction.connections) {
        std::shared_ptr<Road> road = map.roads[jc.incoming_road];
        //use only one lansection for now
        std::shared_ptr<odr::LaneSection> lanesection = *(road->get_lanesections().begin());
        double s = lanesection->get_end(); //temp until I figure out how to get s-value
        //maybe each lane has a different s value depending on the sign of its id
        std::shared_ptr<Lane> min_lane = *(lanesection->id_to_lane.begin());
        std::shared_ptr<Lane> max_lane = *(lanesection->id_to_lane.rbegin());
        double min_t = min_lane.outer_border.get(s);
        double max_t = max_lane.outer_border.get(s);
        Vec3D min_point = min_lane.get_surface_pt(s, min_t);
        Vec3D max_point = max_point.get_surface_pt(s, max_t);

        PointMsg msg_point;
        msg_point.x = min_point.x;
        msg_point.y = min_point.y;
        msg_points.push_back(msg_point);
        
        msg_point.x = max_point.x;
        msg_point.y = max_point.y;
        msg_points.push_back(msg_point);
    }
    //TODO - clockwise the points
    ZoneMsg msg;
    msg.poly.points = msg_points;
    msg.max_speed = 0;
    msg.cost = 0;
    return msg;
}