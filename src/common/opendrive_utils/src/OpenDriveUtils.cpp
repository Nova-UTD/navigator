/*
 * Package:   opendrive_utils
 * Filename:  OpenDriveUtils.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "opendrive_utils/OpenDriveUtils.hpp"
#include "Lanes.h"
#include "RefLine.h"

using namespace navigator::opendrive;
using odr::Vec3D, odr::Lane;

/**
 * @brief loads an Opendrive map from a file name. 
 * 
 * Currently only calls on the OpenDriveMap constructor,
 * but may eventually include map validation and correction.
 * 
 * @param filename 
 * @return std::shared_ptr<odr::OpenDriveMap>
 */
OpenDriveMapPtr navigator::opendrive::load_map(const std::string & filename) {
    OpenDriveMapPtr map = std::make_shared<odr::OpenDriveMap>(filename);

    // Potential map validation and correction here.

    return map;
}


//positive: is the lane_id positive? if so, we have to iterate in the opposite direction
odr::Line3D navigator::opendrive::get_centerline_as_xy(const odr::Lane & lane, double s_start, double s_end, double ds, bool positive)
{
  odr::Line3D result;
    if (positive) {
        for (double s = s_end-ds; s >= s_start; s -= ds) {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s))/2;
	    odr::Vec3D pt = lane.get_surface_pt(s,t);
            result.push_back(pt);
        }
    } else {
        for (double s = s_start; s < s_end; s += ds) {
            double t = (lane.outer_border.get(s) + lane.inner_border.get(s))/2;
	    odr::Vec3D pt = lane.get_surface_pt(s,t);
            result.push_back(pt);
        }
    }
    return result;
}

double navigator::opendrive::get_distance(std::shared_ptr<const odr::RefLine> line, double x, double y)
{
    // Given s, find (x, y) of s, then find distance from s to the input point
    double dist;
    std::function<double(double)> f_dist = [&](const double s)
    {
        const Vec3D pt = line->get_xyz(s);
        dist = odr::euclDistance(odr::Vec2D{pt[0], pt[1]}, {x, y});
        return dist;
    };
    // This is an iterative search method that stops when the matched point is < 0.01 meters = 1 cm
    // It finds the minimum of a function, in this case f_dist, the above distance function,
    // from a start s=0 to s=length, and returns the "s" closest to given (x,y)
    
    //Q: should this return s?
    odr::golden_section_search<double>(f_dist, 0.0, line->length, 1e-2);
    return dist;
}

std::shared_ptr<odr::Lane> navigator::opendrive::get_lane_from_xy(const odr::OpenDriveMap* map, double x, double y)
{
    for (auto road : map->get_roads()) {
        double s = road->ref_line->match(x,y);
        double len = road->length;
        if (s > 0.01 && s-len < -0.01) {
            double t = get_distance(road->ref_line, x,y);
            auto lsec = std::shared_ptr<odr::LaneSection>();
            for (auto s_lsec : road->s_to_lanesection) { // Find the closest lane section
                if (s_lsec.first > s)
                    break;
                lsec = s_lsec.second;
            }

            if (abs(t) > 10) // Too far from road center... This can't be right.
                continue;

            // Find out which side we're on (negative or position)
            std::shared_ptr<Lane> lane_match = lsec->get_lane(s,t);
            auto lane_pt = lane_match->get_surface_pt(s,t);

            // Are we within the lane? If not, try the other side of the road.
            auto lane = lsec->get_lane(s,t);
            if (abs(lane_pt[0]-x) > 0.1 || abs(lane_pt[1]-y) > 0.1) {
                t *= -1;
                
                lane_pt = lane_match->get_surface_pt(s,t);
                if (abs(lane_pt[0]-x) > 0.1 || abs(lane_pt[1]-y) > 0.1) {
                    continue; // Trying other side failed, so we're on the wrong road. Keep searchng.
                }
            }

            lane = lsec->get_lane(s,t);
            double outer_border_t = lane->outer_border.get(s);
            double inner_border_t = lane->inner_border.get(s);
            if (lane->id < 0) {
                if (t < outer_border_t || t > inner_border_t)
                    continue; // Not within lane border
            } else { // Positive lane IDs
                if (t > outer_border_t || t < inner_border_t)
                    continue;
            }

            return lane;
        }
    }
    return std::shared_ptr<odr::Lane>(); // No road found!
}

std::shared_ptr<odr::Lane> navigator::opendrive::get_lane_from_xy_with_route(const odr::OpenDriveMap* map, double x, double y, const std::set<std::string>& rs)
{
    for (auto road : map->get_roads()) {
        double s = road->ref_line->match(x,y);
        double len = road->length;
        if (s > 0.01 && s-len < -0.01) {
            double t = get_distance(road->ref_line, x,y);
            auto lsec = std::shared_ptr<odr::LaneSection>();
            for (auto s_lsec : road->s_to_lanesection) { // Find the closest lane section
                if (s_lsec.first > s)
                    break;
                lsec = s_lsec.second;
            }

            if (abs(t) > 10) // Too far from road center... This can't be right.
                continue;

            // Find out which side we're on (negative or position)
            std::shared_ptr<Lane> lane_match = lsec->get_lane(s,t);
            auto lane_pt = lane_match->get_surface_pt(s,t);

            // Are we within the lane? If not, try the other side of the road.
            auto lane = lsec->get_lane(s,t);
            if (abs(lane_pt[0]-x) > 0.1 || abs(lane_pt[1]-y) > 0.1) {
                t *= -1;
                
                lane_pt = lane_match->get_surface_pt(s,t);
                if (abs(lane_pt[0]-x) > 0.1 || abs(lane_pt[1]-y) > 0.1) {
                    continue; // Trying other side failed, so we're on the wrong road. Keep searchng.
                }
            }

            lane = lsec->get_lane(s,t);
            double outer_border_t = lane->outer_border.get(s);
            double inner_border_t = lane->inner_border.get(s);
            if (lane->id < 0) {
                if (t < outer_border_t || t > inner_border_t)
                    continue; // Not within lane border
            } else { // Positive lane IDs
                if (t > outer_border_t || t < inner_border_t)
                    continue;
            }

            auto road_id = lane->road.lock()->id;
            if (rs.find(road_id) == rs.end()) {
                continue;
            }

            return lane;
        }
    }
    return std::shared_ptr<odr::Lane>(); // No road found!
}