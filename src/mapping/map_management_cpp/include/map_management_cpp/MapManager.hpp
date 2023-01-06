/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

/*
    Currently, this node gets an Evidential Grid Map input from perception and creates a cost map accordingly to ensure safety and good driving practices.
    It does so by giving collisions an extremely high costs and making spaces closer to the next waypoint less costly. *TO BE CONTINUED*
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

#include "rclcpp/rclcpp.hpp"

// Message includes
#include "carla_msgs/msg/carla_world_info.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;
using carla_msgs::msg::CarlaWorldInfo;
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

        private:
            void clock_cb(Clock::SharedPtr);
            void world_info_cb(CarlaWorldInfo::SharedPtr msg);

            rclcpp::Subscription<Clock>::SharedPtr clock_sub;
            rclcpp::Subscription<CarlaWorldInfo>::SharedPtr world_info_sub;

            odr::OpenDriveMap map_ = odr::OpenDriveMap("map_string.xml");
        };
    }
}
