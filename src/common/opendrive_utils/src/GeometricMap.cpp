/*
 * Package:   opendrive_utils
 * Filename:  GeometricMap.cpp
 * Author:    Joshua Williams, Egan Johnson
 * Email:     joshmackwilliams@protonmail.com, eganjohnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <limits>
#include <algorithm>
#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "opendrive_utils/GeometricMap.hpp"


using namespace navigator::opendrive;
using odr::OpenDriveMap;

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;


/**
 * Converts a lane to a boost polygon.
 */
boost_polygon navigator::opendrive::to_boost_polygon(const LanePtr lane){
    boost_polygon gon;

    LaneSectionPtr lane_section = lane->lane_section.lock();
    if (!lane_section)
        throw std::runtime_error("could not access parent lane section");

    double s0 = lane_section->s0;
    double s1 = lane_section->get_end();
    odr::Mesh3D mesh = lane->get_mesh(s0, s1, 1);

    std::vector<boost_point> points;
    // Split by odd/even since mesh alternates left, right, left, right, etc.
    for (int parity : {0, 1})
    {
        for (int i = parity; i < mesh.vertices.size(); i += 2)
        {
            auto vertex = mesh.vertices[i];
            boost::geometry::append(gon.outer(),boost_point(vertex[0], vertex[1]));
        }
    }
    boost::geometry::correct(gon);

    return gon;
}

bool navigator::opendrive::contains(LanePtr lane, double x, double y, double map_x_offset, double map_y_offset){
    boost_polygon gon = to_boost_polygon(lane);
    // world_point = lane_point + map_offset, so world_point - map_offset = lane_point
    boost_point point(x - map_x_offset, y - map_y_offset);
    return boost::geometry::within(point, gon);
}

GeometricMap::GeometricMap(OpenDriveMapPtr map, int grid_size_meters)
{
    grid_size = grid_size_meters;
    this->map = map;

    for (RoadPtr road : map->get_roads())
    {
        for (LaneSectionPtr section : road->get_lanesections())
        {
            double s0 = section->s0;
            double s1 = section->get_end();
            for (LanePtr lane : section->get_lanes())
            {
                // Add lane to grid. Lanes belong to multiple grids
                // Sample all vertices of the lane to create a bounding box,
                // then add the lane to the grid cells that overlap the bounding box.
                // This may result in a few extra cells that are not used, but this
                // is not a problem.
                double x_min = std::numeric_limits<double>::max();
                double x_max = std::numeric_limits<double>::min();
                double y_min = std::numeric_limits<double>::max();
                double y_max = std::numeric_limits<double>::min();

                for(odr::Vec3D v : lane->get_mesh(s0, s1, 1).vertices){
                    x_max = std::max(x_max, v[0]);
                    x_min = std::min(x_min, v[0]);
                    y_max = std::max(y_max, v[1]);
                    y_min = std::min(y_min, v[1]);
                }

                // Add to grid
                for(int x = static_cast<int>(x_min) / grid_size_meters; x <= static_cast<int>(x_max) / grid_size_meters; x++){
                    for(int y = static_cast<int>(y_min) / grid_size_meters; y <= static_cast<int>(y_max) / grid_size_meters; y++){
                        grid[GridCell(x,y)].emplace(lane);
                    }
                }
            }
        }
    }
}

bool GeometricMap::point_in_lane(LanePtr lane, double x, double y){
    return contains(lane, x, y, map->x_offs, map->y_offs);
}

bool GeometricMap::containing_lanes(double x, double y, std::vector<LanePtr> &results, double max_dist)
{
    // Convert x, y to map coordinates (without offset)
    x = x - map->x_offs;
    y = y - map->y_offs;

    Find the grid cell that contains the point
    int x_cell = trunc(x) / grid_size;
    int y_cell = trunc(y) / grid_size;
    
    // Containing lanes will be in the cell as this point
    std::set<LanePtr> cell_lanes = grid[GridCell(x_cell, y_cell)];
    for(LanePtr lane : cell_lanes){
        if(point_in_lane(lane, x, y)){
            results.push_back(lane);
        }
    }

    return !results.empty();

    // TODO: Find the nearest lane to the point if no containing lanes are found.
}
