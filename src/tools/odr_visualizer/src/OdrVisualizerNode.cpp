#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "odr_visualizer/OdrVisualizerNode.hpp"
/*
PSEUDOCODE

On start:
	1. Read xodr file
	2. Iterate over each lane in each road
		a. For each lane, gather vertices and generate triangles
		b. Put tris in array.
	3. Convert tri array to triangle list Marker message (or MarkerArray?)

Every n seconds:
	1. Publish tri array

*/

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

OdrVisualizerNode::OdrVisualizerNode() : Node("odr_visualizer_node") {

	this->declare_parameter<std::string>("xodr_path", "/home/share/maps/Town10HD_Opt.xodr");
	marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/map/viz", 1);

	map_pub_timer = this->create_wall_timer(1s, std::bind(&OdrVisualizerNode::publishMarkerArray, this));

	// Read map from file, using our path param
	std::string xodr_path = this->get_parameter("xodr_path").as_string();
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	odr::OpenDriveMap odr(xodr_path);
	std::vector<Marker> marker_vec;

	for (std::shared_ptr<odr::Road> road : odr.get_roads()) {
        // printf("road: %s, length: %.2f\n", road->id.c_str(), road->length);
        for (std::shared_ptr<odr::LaneSection> lanesec : road->get_lanesections()) {
            for (std::shared_ptr<odr::Lane> lane : lanesec->get_lanes()) {
				// RCLCPP_INFO(this->get_logger(), "Loaded a lane!");
                odr::Mesh3D lane_mesh = lane->get_mesh(lanesec->s0, lanesec->get_end(), 0.1);
				// printf("%i\n", lane_mesh.vertices.size());
				// for (auto vector : lane_mesh.vertices) {
					// printf("(%f,%f,%f) \n", vector[0], vector[1], vector[2]);
				// }

				/**
				 * We rely on a small lib called "polypartition" to convert
				 * our lanelet polygon into triangles. Rviz only accepts
				 * triangles, not more complex polygons.
				 * See: https://en.wikipedia.org/wiki/Tessellation_(computer_graphics)
				 */
				TPPLPoly *inpoly = new TPPLPoly();
				TPPLPolyList *triangulationResults = new TPPLPolyList();
				inpoly->Init(lane_mesh.vertices.size());
				for(uint i=0; i < lane_mesh.vertices.size(); i++) {
					inpoly->GetPoint(i).x = lane_mesh.vertices[i][0]; // x
					inpoly->GetPoint(i).y = lane_mesh.vertices[i][1]; // y
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

				Marker triangleList;

				// Set message header
				triangleList.header.stamp = now();
				triangleList.header.frame_id = "map";
				triangleList.ns = "lanelets"; // Set namespace of viz message
				triangleList.id = lane->id; // Set unique ID of viz msg

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
				marker_vec.push_back(triangleList);
            }
        }
    }
	printf("%i\n", marker_vec.size());

	lane_markers.markers = marker_vec;
	marker_pub->publish(lane_markers);
}

void OdrVisualizerNode::publishMarkerArray() {
	marker_pub->publish(lane_markers);
}