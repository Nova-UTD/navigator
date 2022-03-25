# include "zone_lib/zone.hpp"

using PointMsg = geometry_msgs::msg::Point32;
using ZoneMsg = voltron_msgs::msg::Zone;

namespace navigator::zones_lib
{

ZoneMsg to_zone_msg(const boost_polygon& polygon) {
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

boost_polygon to_boost_polygon(const ZoneMsg& zone) {
    boost_polygon output;
    std::vector<boost_point> points;
    for (auto p : zone.poly.points) {
        boost::geometry::append(output.outer(), boost_point(p.x,p.y));
    }
    //just makes sure the points are in clockwise order
    boost::geometry::correct(output);
    return output;
}


}