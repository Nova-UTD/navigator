#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cstdlib> 

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::ColorRGBA;
using voltron_msgs::msg::FinalPath;

#include "rrt/RRTNode.hpp"



RRTNode::RRTNode() : Node("rrt_node"){
	//cost_map_sub = this -> create_subscription<>
	/*odom_sub = this->create_subscription<Odometry>("/odometry/filtered", 1, [this](Odometry::SharedPtr msg) {
		cached_odom = msg;
	});*/
	//path_pub = this->create_publisher<WayPointPath>("path_pub",1);
	
	// expect the cost map from  cost_map_sub to be passed through
	// expect current position to be passed from odom_sub
	this->path = createTree();
}


std::vector<WayPointRRT> RRTNode:createTree(){
	//have set of states possible actions
	//use costmap to determine which state transisitoned would be the best
	// use heuristic to determine which path will be the best
	return NULL;
}


