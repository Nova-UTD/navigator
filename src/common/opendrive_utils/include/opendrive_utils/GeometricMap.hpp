/*
 * Package:   opendrive_utils
 * Filename:  GeometricMap.hpp
 * Author:    Joshua Williams, Egan Johnson
 * Email:     joshmackwilliams@protonmail.com, eganjohnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once
#include "OpenDriveMap.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <opendrive_utils/OpenDriveUtils.hpp>
#include <opendrive_utils/OpenDriveTypes.hpp>

namespace navigator
{
  namespace opendrive
  {
    using namespace types;
    typedef boost::geometry::model::d2::point_xy<double> boost_point;
    typedef boost::geometry::model::polygon<boost_point> boost_polygon;
    /**
     * @brief Checks if a lane contains a point
     * 
     * x and y are in odr::OpenDriveMap coordinates.
     * generates a mesh and polygon for the lane every call
     * 
     * @param lane 
     * @param x 
     * @param y 
     * @return true 
     * @return false 
     */
    bool contains(LanePtr lane, double x, double y, double map_x_offset, double map_y_offset);
    /**
     * @brief Checks if a polygon contains a point. converts coordinates
     * 
     * x and y are in odr::OpenDriveMap coordinates.
     * 
     * @param lane 
     * @param x 
     * @param y 
     * @return true 
     * @return false 
     */
    bool contains(const boost_polygon& gon, double x, double y, double map_x_offset, double map_y_offset);

    

    boost_polygon to_boost_polygon(const odr::Mesh3D& mesh);

    class GeometricMap final
    {
    
    public:
      /**
       * @brief Construct a new Geometric Map object from an OpenDriveMap.
       * 
       * When the map is constructed, lanes are added to the map in a grid
       * with specified cell size, default 100m. The grid is important when
       * looking up the lane that a point lies on. Smaller grid size will
       * result in faster lookup times, but will also result in the grid
       * taking up more memory. 
       * 
       * @param map 
       * @param grid_size_meters 
       */
      GeometricMap(types::OpenDriveMapPtr map, int grid_size_meters = 100);

      /**
       * @brief Finds all lanes containing the point. If no lanes are found, finds 
       *  the closest lane within a certain distance.
       * 
       *  Coordinates and measures are in the odr::OpenDriveMap's coordinate system, pre-offset
       * 
       * @param x - x coordinate of the point
       * @param y - y coordinate of the point
       * @param results - vector of lanes to hold the results
       * @param max_dist - maximum distance to search for lanes, default 100m
       * @return true if lanes were found containing the point
       * @return false if no lanes were found containing the point. results may still
       *  contain a single nearest lane.
       */
      bool containing_lanes(double x, double y, std::vector<LanePtr> &results, double max_dist = 100);

      /**
       * @brief Determines if the point is contained within the lane
       * 
       * Coordinates and measures are in the odr::OpenDriveMap's coordinate system, pre-offset
       * 
       * @param x 
       * @param y 
       * @param lane 
       * @return true 
       * @return false 
       */
      bool point_in_lane(LanePtr lane, double x, double y);


    private:
      typedef std::pair<int, int> GridCell;

      // The grid of lanes.
      std::map<GridCell, std::set<LanePtr>> grid;
      int grid_size;

      // Store a reference to the original map
      OpenDriveMapPtr map;
    };
  }
}
