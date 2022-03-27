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

using namespace navigator::opendrive;

/**
 * @brief loads an Opendrive map from a file name.
 *
 * Currently only calls on the OpenDriveMap constructor,
 * but may eventually include map validation and correction.
 *
 * @param filename
 * @return std::shared_ptr<odr::OpenDriveMap>
 */
OpenDriveMapPtr navigator::opendrive::load_map(const std::string &filename)
{
    OpenDriveMapPtr map = std::make_shared<odr::OpenDriveMap>(filename, true, true, false, true);

    // Potential map validation and correction here.

    return map;
}

LanePtr navigator::opendrive::get_lane_from_xy(OpenDriveMapPtr map, double x, double y)
{
    for (auto road : map->get_roads())
    {
        double s = road->ref_line->match(x, y);
        double len = road->length;
        if (s > 0.01 && s - len < -0.01)
        {
            double t = road->ref_line->get_distance(x, y);
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
                std::cout << "FAR:  dx: " << lane_pt[0] - x << ", dy: " << abs(lane_pt[1] - y) << std::endl;
                t *= -1;

                lane_pt = lane_match->get_surface_pt(s, t);
                if (abs(lane_pt[0] - x) > 0.1 || abs(lane_pt[1] - y) > 0.1)
                {
                    std::cout << "FAIL: dx: " << lane_pt[0] - x << ", dy: " << abs(lane_pt[1] - y) << std::endl;
                    continue; // Trying other side failed, so we're on the wrong road. Keep searchng.
                }
                else
                {
                    std::cout << "FIX:  dx: " << lane_pt[0] - x << ", dy: " << abs(lane_pt[1] - y) << std::endl;
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
            std::cout << "OK:   (" << outer_border_t << " < " << t << " > " << inner_border_t << ") dx: " << lane_pt[0] - x << ", dy: " << abs(lane_pt[1] - y) << std::endl;

            return lane;
        }
    }
    return std::shared_ptr<odr::Lane>(); // No road found!
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
                    std::printf("R%sL%i added to reult\n", lane->type.c_str());
                    result.push_back(lane);
                    nearby_counter++;
                }
            }
        }
        if (nearby_counter > 0)
            std::printf("Road %s has %i nearby lanes\n", road->id.c_str(), nearby_counter);
    }
    std::printf("Returning %i lanes\n", result.size());
    return result;
}
