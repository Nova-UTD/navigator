/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/OpenDriveUtils.hpp"
#include <iostream>
#include "pugixml/pugixml.hpp"

using namespace navigator::opendrive;

/**
 * @brief loads an Opendrive map from a file name.
 *
 * Currently only calls on the OpenDriveMap constructor,
 * and parses additional information (like signals)
 *
 * @param filename
 * @return std::shared_ptr<MapInfo>
 */
std::shared_ptr<MapInfo> navigator::opendrive::load_map(const std::string &filename)
{
    OpenDriveMapPtr map = std::make_shared<odr::OpenDriveMap>(filename, odr::OpenDriveMapConfig{true, true, true, false, true});
    std::unordered_map<std::string, std::vector<Signal>> signals;
    parse_signals(signals, filename);
    // Potential map validation and correction here.
    std::shared_ptr<MapInfo> info = std::make_shared<MapInfo>();
    info->signals = signals;
    info->map = map;
    return info;
}

void navigator::opendrive::parse_signals(std::unordered_map<std::string, std::vector<Signal>>& dst, const std::string &filename) {
    pugi::xml_document xml_doc;
    pugi::xml_parse_result result = xml_doc.load_file(filename.c_str());
    if (!result)
        printf("%s\n", result.description());
    pugi::xml_node odr_node = xml_doc.child("OpenDRIVE");
    for (pugi::xml_node road_node : odr_node.children("road"))
    {
        std::string road_id = road_node.attribute("id").as_string();
        std::vector<Signal> signals;
        pugi::xml_node signals_node = road_node.child("signals");
        for (pugi::xml_node signal_node : signals_node.children("signal")) {
            Signal signal;
            signal.id = signal_node.attribute("id").as_string();
            signal.type = signal_node.attribute("type").as_string();
            signal.name = signal_node.attribute("name").as_string();
            signal.s = std::stod(signal_node.attribute("s").as_string());
            signal.t = std::stod(signal_node.attribute("t").as_string());
            signal.dynamic = signal_node.attribute("dynamic").as_string();
            signal.orientation = signal_node.attribute("orientation").as_string();
            signal.road = road_id;
            signals.push_back(signal);
        }
        if (signals.size() > 0)
            dst[road_id] = signals;
    }
    
}

double navigator::opendrive::get_distance(std::shared_ptr<const odr::RefLine> line, double x, double y)
{
    // Given s, find (x, y) of s, then find distance from s to the input point
    double dist;
    std::function<double(double)> f_dist = [&](const double s)
    {
        const odr::Vec3D pt = line->get_xyz(s);
        dist = odr::euclDistance(odr::Vec2D{pt[0], pt[1]}, {x, y});
        return dist;
    };
    // This is an iterative search method that stops when the matched point is < 0.01 meters = 1 cm
    // It finds the minimum of a function, in this case f_dist, the above distance function,
    // from a start s=0 to s=length, and returns the "s" closest to given (x,y)

    // Q: should this return s?
    odr::golden_section_search<double>(f_dist, 0.0, line->length, 1e-2);
    return dist;
}

LanePtr navigator::opendrive::get_lane_from_xy(OpenDriveMapPtr map, double x, double y)
{
    for (auto road : map->get_roads())
    {
        double s = road->ref_line->match(x, y);
        double len = road->length;
        if (s > 0.01 && s - len < -0.01)
        {
            double t = get_distance(road->ref_line, x, y);
            auto lsec = std::shared_ptr<odr::LaneSection>();
            for (auto s_lsec : road->s_to_lanesection)
            { // Find the closest lane section
                if (s_lsec.first > s)
                    break;
                lsec = s_lsec.second;
            }

            if (abs(t) > 10) // Too far from road center... This can't be right.
                continue;

            // Find out which side we're on (negative or positive)
            std::shared_ptr<odr::Lane> lane_match = lsec->get_lane(s, t);
            auto lane_pt = lane_match->get_surface_pt(s, t);

            // Are we within the lane? If not, try the other side of the road.
            auto lane = lsec->get_lane(s, t);
            if (abs(lane_pt[0] - x) > 0.1 || abs(lane_pt[1] - y) > 0.1)
            {
                t *= -1;

                lane_pt = lane_match->get_surface_pt(s, t);
                if (abs(lane_pt[0] - x) > 0.1 || abs(lane_pt[1] - y) > 0.1)
                {
                    continue; // Trying other side failed, so we're on the wrong road. Keep searchng.
                }
            }

            lane = lsec->get_lane(s, t);
            double outer_border_t = lane->outer_border.get(s);
            double inner_border_t = lane->inner_border.get(s);
            if (lane->id < 0)
            {
                if (t < outer_border_t || t > inner_border_t)
                    continue; // Not within lane border
            }
            else
            { // Positive lane IDs
                if (t > outer_border_t || t < inner_border_t)
                    continue;
            }

            return lane;
        }
    }
    return nullptr; // No road found!
}

std::vector<std::shared_ptr<odr::Lane>> navigator::opendrive::get_nearby_lanes(OpenDriveMapPtr map, double x, double y, double distance)
{
    std::vector<std::shared_ptr<odr::Lane>> result;

    for (auto road : map->get_roads())
    {
        int nearby_counter = 0;
        double s = road->ref_line->match(x, y);

        for (auto lsec : road->get_lanesections())
        {
            for (auto lane : lsec->get_lanes())
            {
                double t = lane->outer_border.get(s);
                odr::Vec3D border_pt = lane->get_surface_pt(s, t);

                if ((std::abs(border_pt[0] - x) < distance) & (std::abs(border_pt[1] - y) < distance))
                {
                    result.push_back(lane);
                    nearby_counter++;
                }
            }
        }
    }
    return result;
}

bool navigator::opendrive::signal_applies(const Signal& signal, double s, const std::string& my_road_id, int my_lane_id)
{
    //checking if road id matches is important to not only to make sure we are on the same road as the sign,
    //but also to make sure orientation matches (- on sign = negative lane id)
    if (signal.road != my_road_id) {
        return false; //signal is not on this road
    }
    //each signal has an s value that determines where on the road it is, which the path point should be close enough to,
    //and an orientation (relative to road, so matches lane id). we should be traveling in the same orientation
    constexpr double s_tolerance = 1;
    if (abs(signal.s - s) > s_tolerance) {
        return false; //too far from signal
    }
    //check orientation
    if (signal.orientation == "none" 
        || (signal.orientation == "+" && my_lane_id > 0)
        || (signal.orientation == "-" && my_lane_id < 0)) {
        //we are going the correct way
        return true;
    }
    return false;
}

// positive: is the lane_id positive? if so, we have to iterate in the opposite direction
odr::Line3D navigator::opendrive::get_centerline_as_xy(const odr::Lane &lane, double s_start, double s_end, double ds, bool positive)
{
    odr::Line3D result;
    if (positive)
    {
        for (double s = s_end - ds; s >= s_start; s -= ds)
        {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s)) / 2;
            odr::Vec3D pt = lane.get_surface_pt(s, t);
            result.push_back(pt);
        }
    }
    else
    {
        for (double s = s_start; s < s_end; s += ds)
        {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s)) / 2;
            odr::Vec3D pt = lane.get_surface_pt(s, t);
            result.push_back(pt);
        }
    }
    return result;
}

LanePtr navigator::opendrive::get_lane_from_xy_with_route(OpenDriveMapPtr map, double x, double y, const std::set<std::string> &rs)
{
    for (auto road : map->get_roads())
    {
        double s = road->ref_line->match(x, y);
        double len = road->length;
        if (s > 0.01 && s - len < -0.01)
        {
            double t = get_distance(road->ref_line, x, y);
            auto lsec = std::shared_ptr<odr::LaneSection>();
            for (auto s_lsec : road->s_to_lanesection)
            { // Find the closest lane section
                if (s_lsec.first > s)
                    break;
                lsec = s_lsec.second;
            }

            if (abs(t) > 10) // Too far from road center... This can't be right.
                continue;

            // Find out which side we're on (negative or position)
            std::shared_ptr<odr::Lane> lane_match = lsec->get_lane(s, t);
            auto lane_pt = lane_match->get_surface_pt(s, t);

            // Are we within the lane? If not, try the other side of the road.
            auto lane = lsec->get_lane(s, t);
            if (abs(lane_pt[0] - x) > 0.1 || abs(lane_pt[1] - y) > 0.1)
            {
                t *= -1;

                lane_pt = lane_match->get_surface_pt(s, t);
                if (abs(lane_pt[0] - x) > 0.1 || abs(lane_pt[1] - y) > 0.1)
                {
                    continue; // Trying other side failed, so we're on the wrong road. Keep searchng.
                }
            }

            lane = lsec->get_lane(s, t);
            double outer_border_t = lane->outer_border.get(s);
            double inner_border_t = lane->inner_border.get(s);
            if (lane->id < 0)
            {
                if (t < outer_border_t || t > inner_border_t)
                    continue; // Not within lane border
            }
            else
            { // Positive lane IDs
                if (t > outer_border_t || t < inner_border_t)
                    continue;
            }

            auto road_id = lane->road.lock()->id;
            if (rs.find(road_id) == rs.end())
            {
                continue;
            }

            return lane;
        }
    }
    return std::shared_ptr<odr::Lane>(); // No road found!
}
