/*
 * Package:   map_management
 * Filename:  MapManager.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <chrono> // Time literals
#include <vector>
#include <string>
#include <deque>

// libOpenDRIVE includes
#include "OpenDriveMap.h"
#include "Lane.h"
#include "Road.h"
#include "RoadNetworkMesh.h"
#include "Geometries/Line.h"
#include "RefLine.h"

// Boost
#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/io/wkt/read.hpp>

// LEMON
#include "lemon/smart_graph.h"
#include <lemon/dijkstra.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Message includes
#include "navigator_msgs/msg/carla_route.hpp"
#include "carla_msgs/msg/carla_world_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigator_msgs/srv/set_route.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/float32.hpp"

#include "map_management/RouteManager.hpp"

using namespace std::chrono_literals;
using namespace nav_msgs::msg;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using navigator_msgs::msg::CarlaRoute;
using carla_msgs::msg::CarlaWorldInfo;
using PointMsg = geometry_msgs::msg::Point;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using rosgraph_msgs::msg::Clock;

using SptSolver = lemon::Dijkstra<lemon::SmartDigraph, lemon::SmartDigraph::ArcMap<double>>;
using lemon::SmartDigraph;
using std::string;

namespace navigator
{
    namespace planning
    {

        class MapManagementNode : public rclcpp::Node
        {
        public:
            // Constructor
            MapManagementNode();

            // Functions
            void publishGrids(int top, int bottom, int side, float res);

        private:
            // Parameters
            // TODO: Convert to ros params
            std::chrono::milliseconds GRID_PUBLISH_FREQUENCY = 3000ms;
            std::chrono::milliseconds LOCAL_ROUTE_LS_FREQ = 1000ms; 
            std::chrono::milliseconds SMOOTH_ROUTE_LS_FREQ = 1000ms;
            const int GRID_RANGE = 30;
            const float GRID_RES = 0.4;

            void clockCb(Clock::SharedPtr msg);
            TransformStamped getEgoTf();
            void drivableAreaGridPubTimerCb();
            void publishRefinedRoute();
            void publishSmoothRoute();
            void updateRouteWaypoints(Path::SharedPtr msg);
            std::vector<odr::LaneKey> calculateRoute(odr::LaneKey start, odr::LaneKey end);
            lemon::SmartDigraph *g = nullptr;

            lemon::SmartDigraph::NodeMap<odr::LaneKey> *nodeMap;
            std::map<odr::LaneKey, int> *routing_nodes_ = nullptr;
            lemon::SmartDigraph::ArcMap<double> *costMap = nullptr;

            std::vector<odr::LaneKey> getTrueSuccessors(odr::LaneKey key);
            void setRoute(const std::shared_ptr<navigator_msgs::srv::SetRoute::Request> request, std::shared_ptr<navigator_msgs::srv::SetRoute::Response> response);
            void setRouteFromClickedPt(const PointStamped clicked_pt);
            void recursiveSearch(std::vector<odr::LaneKey> predecessors, odr::LaneKey target);
            void buildTrueRoutingGraph();
            void updateLocalRouteLinestring();
            odr::RoutingGraph faulty_routing_graph;
            odr::point getLaneStart(odr::LaneKey key);

            std::unordered_map<odr::LaneKey, std::vector<odr::LaneKey>> connectivity_map;

            void updateRouteGivenDestination(odr::point destination);
            void worldInfoCb(CarlaWorldInfo::SharedPtr msg);
            std::map<odr::LaneKey, bool> getJunctionMap(std::vector<odr::LanePair> lane_polys);

            LineString getLaneCenterline(odr::LaneKey key);
            std::vector<LineString> getCenterlinesFromKeys(std::vector<odr::LaneKey> keys, odr::RoutingGraph graph);

            void setPredeterminedRoute();

            rclcpp::Publisher<OccupancyGrid>::SharedPtr drivable_grid_pub_;
            rclcpp::Publisher<OccupancyGrid>::SharedPtr junction_grid_pub_;
            rclcpp::Publisher<OccupancyGrid>::SharedPtr route_dist_grid_pub_;
            rclcpp::Publisher<Path>::SharedPtr route_path_pub_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr route_progress_pub_;
            rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;
            rclcpp::Subscription<Clock>::SharedPtr clock_sub;
            rclcpp::Subscription<Path>::SharedPtr rough_path_sub_;
            rclcpp::Subscription<CarlaWorldInfo>::SharedPtr world_info_sub;
            rclcpp::Subscription<PointStamped>::SharedPtr clicked_point_sub_; // For RViz interface
            void clickedPointCb(PointStamped::SharedPtr msg);

            rclcpp::Service<navigator_msgs::srv::SetRoute>::SharedPtr route_set_service_;

            rclcpp::TimerBase::SharedPtr drivable_area_grid_pub_timer_;
            rclcpp::TimerBase::SharedPtr route_distance_grid_pub_timer_;
            rclcpp::TimerBase::SharedPtr route_timer_;
            rclcpp::TimerBase::SharedPtr smooth_route_timer_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            Clock::SharedPtr clock_;
            odr::OpenDriveMap *map_ = nullptr;
            std::vector<odr::LanePair> lane_polys_;
            std::vector<odr::Lane> lanes_in_route_;
            std::map<odr::LaneKey, bool> road_in_junction_map_;
            Path smoothed_route_msg_;
            LineString rough_route_;
            Path rough_route_msg_;
            bg::model::multi_linestring<bg::model::linestring<odr::point>> route_linestrings_;
            bg::model::linestring<odr::point> route_linestring_;
            bg::model::linestring<odr::point> local_route_linestring_;
            bg::model::multi_linestring<bg::model::linestring<odr::point>> local_route_linestrings_;
            bgi::rtree<odr::value, bgi::rstar<16, 4>> map_wide_tree_;
            bgi::rtree<odr::value, bgi::rstar<16, 4>> rough_route_tree_;

            std::vector<odr::polygon> junction_polys_;

            RouteManager rm;
        };
    }
}