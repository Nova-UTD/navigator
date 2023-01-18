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

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Message includes
#include "carla_msgs/msg/carla_route.hpp"
#include "carla_msgs/msg/carla_world_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;
using namespace nav_msgs::msg;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using carla_msgs::msg::CarlaRoute;
using carla_msgs::msg::CarlaWorldInfo;
using PointMsg = geometry_msgs::msg::Point;
using geometry_msgs::msg::TransformStamped;
using rosgraph_msgs::msg::Clock;

namespace navigator
{
    namespace perception
    {

        class MapManagementNode : public rclcpp::Node
        {
        public:
            // Constructor
            MapManagementNode();

            // Functions
            OccupancyGrid getDrivableAreaGrid(PointMsg center, int range, float res);

        private:
            // Parameters
            // TODO: Convert to ros params
            std::chrono::milliseconds GRID_PUBLISH_FREQUENCY = 200ms;
            const int GRID_RANGE = 40;
            const float GRID_RES = 0.4;

            void clockCb(Clock::SharedPtr msg);
            TransformStamped getVehicleTf();
            void gridPubTimerCb();
            void getLanesFromRoughPath(Path::SharedPtr msg);
            void updateRoute();
            void worldInfoCb(CarlaWorldInfo::SharedPtr msg);

            rclcpp::Publisher<OccupancyGrid>::SharedPtr grid_pub_;
            rclcpp::Subscription<Clock>::SharedPtr clock_sub;
            rclcpp::Subscription<Path>::SharedPtr rough_path_sub_;
            rclcpp::Subscription<CarlaWorldInfo>::SharedPtr world_info_sub;

            rclcpp::TimerBase::SharedPtr grid_pub_timer_;
            rclcpp::TimerBase::SharedPtr route_update_timer_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            Clock::SharedPtr clock_;
            odr::OpenDriveMap *map_ = nullptr;
            std::vector<odr::LanePair> lane_polys_;
            std::vector<odr::Lane> lanes_in_route_;
            bgi::rtree<odr::value, bgi::rstar<16, 4>> map_wide_tree_;
        };
    }
}
