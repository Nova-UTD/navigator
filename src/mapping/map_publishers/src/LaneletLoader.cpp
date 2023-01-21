/*
 * Package:   map_publishers
 * Filename:  GPSInterfaceNode.cpp
 * Author:    Will Heitman
 * Email:     Will.Heitman@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/geometry/Polygon.h>
#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"
#include "polypartition.h"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
/**
 * @brief Loads a lanelet .osm file and publishes a visualization
 */
class LaneletLoader : public rclcpp::Node
{
  public:
    LaneletLoader()
    : Node("lanelet_loader")
    {
        this->declare_parameter<std::string>("osm_path", "/home/main/voltron/assets/grand_loop.osm");
        _map = load_from_osm();
        _lanelet_viz_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/viz/lanelets", 10);
        RCLCPP_INFO_STREAM(this->get_logger(), "Map has "<<_map->laneletLayer.size()<<" lanelets");
        visualizeLanelets(_map);
    }

  private:
    /**
     * @brief Reads osm from path specified in osm_path param
     * @return Pointer to loaded lanelet map
     */
    lanelet::LaneletMapPtr load_from_osm() {
        using namespace lanelet;
        this->get_parameter("osm_path", osm_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Reading map from "<<osm_path_);
        projection::UtmProjector projector(Origin({32.9903275000000, -96.7522478333333}));  // we will go into details later
        return load(osm_path_, projector);
    }

    /**
     * @brief Publishes lanelets as a Marker message
     * 
     * @param map_ptr The lanelet map
     */
    void visualizeLanelets(lanelet::LaneletMapPtr map_ptr) {
      using namespace lanelet;
      using namespace visualization_msgs::msg;
      using namespace geometry_msgs::msg;
      using namespace std_msgs::msg;
      for(auto lanelet: map_ptr->laneletLayer) {
        CompoundPolygon2d lanelet_poly = lanelet.polygon2d();

        auto basic_poly = lanelet_poly.basicPolygon();
        Marker triangleList;

        /**
         * We rely on a small lib called "polypartition" to convert
         * our lanelet polygon into triangles. Rviz only accepts
         * triangles, not more complex polygons.
         * See: https://en.wikipedia.org/wiki/Tessellation_(computer_graphics)
         */
        TPPLPoly *inpoly = new TPPLPoly();
        TPPLPolyList *triangulationResults = new TPPLPolyList();
        inpoly->Init(basic_poly.size());
        for(uint i=0; i<basic_poly.size(); i++) {
          inpoly->GetPoint(i).x = basic_poly[i][0];
          inpoly->GetPoint(i).y = basic_poly[i][1];
        }
        // Orientation of vertices must be set for algorithm to work.
        inpoly->SetOrientation(TPPLOrientation::TPPL_ORIENTATION_CCW);
        TPPLPartition partitioner;

        // Triangulate using ear clipping
        partitioner.Triangulate_EC(inpoly, triangulationResults);

        // Construct an array of Point messages
        std::vector<Point> triangle_points;
        for(TPPLPoly triangle : *triangulationResults) {
          Point a;
          a.x = triangle[0].x;
          a.y = triangle[0].y;
          triangle_points.push_back(a);
          a.x = triangle[1].x;
          a.y = triangle[1].y;
          triangle_points.push_back(a);
          a.x = triangle[2].x;
          a.y = triangle[2].y;
          triangle_points.push_back(a);
        }

        // Set message header
        triangleList.header.stamp = now();
        triangleList.header.frame_id = "map";
        triangleList.ns = "lanelets"; // Set namespace of viz message
        triangleList.id = lanelet.id(); // Set unique ID of viz msg

        // Marker type
        triangleList.type = triangleList.TRIANGLE_LIST;
        triangleList.action = triangleList.MODIFY;
        triangleList.points = triangle_points;

        // Create vector for message scale
        Vector3 scale_vector;
        scale_vector.x = 1;
        scale_vector.y = 1;
        scale_vector.z = 1;
        triangleList.scale = scale_vector;

        // Set color (blue-gray)
        ColorRGBA color;
        color.r = .588;
        color.g = .675;
        color.b = .718;
        color.a = .8;
        triangleList.color = color;

        // Ask Rviz to move our marker with the camera
        triangleList.frame_locked = true;

        _lanelet_viz_publisher->publish(triangleList);
      }
    }

    std::string osm_path_;
    lanelet::LaneletMapPtr _map;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _lanelet_viz_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneletLoader>());
  rclcpp::shutdown();
  return 0;
}