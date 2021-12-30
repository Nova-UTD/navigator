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

/*

PSEUDOCODE
On init, we want to:
1. Read filename from param
2. Read pub period from param
3. Load map using lanelet2_io into memory
4. Convert each lanelet into an array of triangles
5. Save all triangles as a Triangle List https://wiki.ros.org/rviz/DisplayTypes/Marker#Triangle_List_.28TRIANGLE_LIST.3D11.29_.5B1.1.2B-.5D

Every n seconds (pub period from param):
1. Publish the triangle list to /viz/lanelets

*/

class LaneletLoader : public rclcpp::Node
{
  public:
    LaneletLoader()
    : Node("lanelet_loader_node")
    {
        this->declare_parameter<std::string>("osm_path", "/home/main/voltron/assets/grand_loop.osm");
        _map = load_from_osm();
        _lanelet_viz_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/viz/lanelets", 10);
        RCLCPP_INFO_STREAM(this->get_logger(), "Map has "<<_map->laneletLayer.size()<<" lanelets");
        visualizeLanelets(_map);
    }

  private:
    lanelet::LaneletMapPtr load_from_osm() {
        using namespace lanelet;
        this->get_parameter("osm_path", osm_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Reading map from "<<osm_path_);
        projection::UtmProjector projector(Origin({32.9903275000000, -96.7522478333333}));  // we will go into details later
        return load(osm_path_, projector);
    }

    void visualizeLanelets(lanelet::LaneletMapPtr map_ptr) {
      using namespace lanelet;
      using namespace visualization_msgs::msg;
      using namespace geometry_msgs::msg;
      using namespace std_msgs::msg;
      for(auto lanelet: map_ptr->laneletLayer) {
        CompoundPolygon2d lanelet_poly = lanelet.polygon2d();
        // std::vector<Point> vertices;
        // for(auto point : lanelet_poly) {
        //   RCLCPP_INFO_STREAM(this->get_logger(), point);
        //   vertices.push_back(Point(point.x(),point.y()));
        // }
        // 
        // std::vector<point_data> vertices = lanelet_poly.points();

        auto basic_poly = lanelet_poly.basicPolygon();
        Marker triangleList;
        RCLCPP_INFO_STREAM(this->get_logger(), "Polygon points: "<<basic_poly.size());
        // auto triangles = lanelet::geometry::convexPartition(lanelet_poly);
        // RCLCPP_INFO_STREAM(this->get_logger(), lanelet_poly.size());
        // RCLCPP_INFO_STREAM(this->get_logger(), "Triangles: "<<triangles.size());
        

        TPPLPoly *inpoly = new TPPLPoly();
        TPPLPolyList *triangulationResults = new TPPLPolyList();
        inpoly->Init(basic_poly.size());
        for(uint i=0; i<basic_poly.size(); i++) {
          // RCLCPP_INFO_STREAM(this->get_logger(), point[0]<<" "<<point[1]);
          inpoly->GetPoint(i).x = basic_poly[i][0];
          inpoly->GetPoint(i).y = basic_poly[i][1];
        }
        inpoly->SetOrientation(TPPLOrientation::TPPL_ORIENTATION_CCW);
        TPPLPartition partitioner;
        partitioner.Triangulate_EC(inpoly, triangulationResults);

        
        RCLCPP_INFO_STREAM(this->get_logger(), std::endl<<std::endl);

        // Construct an array of Point messages
        std::vector<Point> triangle_points;
        for(TPPLPoly triangle : *triangulationResults) {
          RCLCPP_INFO_STREAM(this->get_logger(), triangle[0].x<<" "<<triangle[0].y<<" "
                                               <<triangle[1].x<<" "<<triangle[1].y<<" "
                                               <<triangle[2].x<<" "<<triangle[2].y);
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

        triangleList.type = triangleList.TRIANGLE_LIST;
        triangleList.action = triangleList.MODIFY;
        triangleList.points = triangle_points;

        // Create vector for message scale
        Vector3 scale_vector;
        scale_vector.x = 1;
        scale_vector.y = 1;
        scale_vector.z = 1;
        triangleList.scale = scale_vector;

        // Set color
        ColorRGBA color;
        color.r = 1;
        color.g = 1;
        color.b = 1;
        color.a = 1;
        triangleList.color = color;

        _lanelet_viz_publisher->publish(triangleList);

        // for (const auto& vertex: vd.vertices()) {
        //     std::vector<point_data> triangle;
        //     auto edge = vertex.incident_edge();
        //     do {
        //         auto cell = edge->cell();
        //         assert(cell->contains_point());

        //         triangle.push_back(vertices[cell->source_index()]);
        //         if (triangle.size() == 3) {
        //             // process output triangles
        //             std::cout << "Got triangle:" << triangle << std::endl;
        //             triangle.erase(triangle.begin() + 1);
        //         }

        //         edge = edge->rot_next();
        //     } while (edge != vertex.incident_edge());
        // }
        
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