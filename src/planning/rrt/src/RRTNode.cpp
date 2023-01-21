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
#include <set>
#include <unordered_set>
#include <utility>
#include <random>
#include<cmath>
#include <limits>
#include "rrt/RRTNode.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;
using nav_msgs::msg::Path;
using builtin_interfaces::msg::Time;
using nova_msgs::msg::Egma;
using nova_msgs::msg::EvidentialGrid;
using nova_msgs::msg::EvidentialGridOccupancy;
using nova_msgs::msg::GoalPosition;
using nova_msgs::msg::RrtPath;
using std::placeholders::_1;

//Mock occupancy grid

RRTNode::RRTNode() : Node("rrt_node") {

	this->goal.x = (-1);
	this->goal.y = (-1);
	this->goal.index = (-1);

	//this->fakeCostMapPub = this->create_publisher<Egma>("/planning/cost_map", 10);
	//this->fakeGoalPup = this->create_publisher<GoalPosition>("/planning/goal_position", 10);


	//this->rrt_path_pub = this->create_publisher<RrtPath>("/planning/rrt_path_temp", 10);
	this->rrt_path_publisher = this->create_publisher<Path>("/planning/rrt_path", 10);
  	this->cost_map_sub = this->create_subscription<Egma>("/planning/cost_map", 5, std::bind(&RRTNode::findPath, this, _1));
	/*this->goal_position_sub = this->create_subscription<GoalPosition>("/planning/goal_position", 1, [this](GoalPosition::SharedPtr msg) {
		this->goal.x = (int)msg->goal_point.x;
		this->goal.y = (int)msg->goal_point.y;
		this->goal.index = (-1);
		RCLCPP_WARN(this->get_logger(), "goal published2: x: %i y: %i index: %i", this->goal.x, this->goal.y, this->goal.index);
	});*/
/*
	Egma fakeEgma;
	
	for(int i =0; i< 10; i++){
		EvidentialGrid tempGrid;
		for(int j = 0; j< 11; j++){
			EvidentialGridOccupancy tempRow;
			
			tempRow.occupancy_value.push_back((float)1);
			tempRow.occupancy_value.push_back((float)1);
			tempRow.occupancy_value.push_back((float)1);		
			tempRow.occupancy_value.push_back((float)0);
			tempRow.occupancy_value.push_back((float)0);
			tempRow.occupancy_value.push_back((float)0);
			tempRow.occupancy_value.push_back((float)1);
			tempRow.occupancy_value.push_back((float)1);
			tempRow.occupancy_value.push_back((float)1);
			tempGrid.grid.push_back(tempRow);
		}
		tempGrid.header.stamp.sec = i;
		fakeEgma.egma.push_back(tempGrid);
	}

	fakeEgma.goal_point.x = 0;
	fakeEgma.goal_point.y = 5;
	fakeEgma.goal_point.z = 0;

	this->fakeCostMapPub->publish(fakeEgma);
	//RCLCPP_WARN(this->get_logger(), "egma published");
*/
/*
	GoalPosition fakeGoal;
	fakeGoal.goal_point.x = (float)0;
	fakeGoal.goal_point.y = (float)5;
	this->fakeGoalPup->publish(fakeGoal);*/
	//RCLCPP_WARN(this->get_logger(), "goal published: x: %.6f y: %.6f", fakeGoal.goal_point.x, fakeGoal.goal_point.y);
	
}

void RRTNode::findRandomPair(int gridSize_x, int gridSize_y){
	int i = rand() % gridSize_x;
	int j = rand() % gridSize_y;
	this->randomPoint = std::make_pair(i,j);
	return;
}

void RRTNode::findClosestState(TreeNode *head){
	if(head == nullptr){
		return;
	}
	
	float temp_min = pow(head->x - this->randomPoint.first,2) + pow(head->y - this->randomPoint.second,2);
	
	if(temp_min < this->currentMinCostForSingleNode && head->x>=randomPoint.first){
		this->currentMinCostForSingleNode = temp_min;
		this->closest = head;
	}

	for(int i=0; i<(int)head->children.size();i++){
		findClosestState(&head->children[i]);
	}

	return;
}

void RRTNode::addNewRRTNode(Egma::SharedPtr map){
	std::pair<int,int> best(this->currentPosition.x, this->currentPosition.y);

	findRandomPair(map->egma[0].grid.size(), map->egma[0].grid[0].occupancy_value.size());
	//RCLCPP_WARN(this->get_logger(), "random point: x: %i, y: %i", this->randomPoint.first, this->randomPoint.second);

	this->currentMinCostForSingleNode = std::numeric_limits<float>::max();
	findClosestState(&this->currentPosition);
	//RCLCPP_WARN(this->get_logger(), "closest state: x: %i, y: %i index: %i", this->closest->x, this->closest->y, this->closest->index);
	//RCLCPP_WARN(this->get_logger(), "closest state address:  %p", (void*)&closest);

	float nodeMinOccupancyValue = 1;
	int k = this->closest->index+1;

	if(k >= (int)map->egma.size()){
		return;
	}

	int column = this->closest->y-this->maxDistanceToExplore;
	if(column < 0){
		column = 0;
	}
	
	for (int i=this->closest->x; i>=(this->closest->x-this->maxDistanceToExplore) && i>=0; i--){
		for(int j=column; j<(this->closest->y+this->maxDistanceToExplore) && j<= (int)map->egma[k].grid[i].occupancy_value.size(); j++){
			if (map->egma[k].grid[i].occupancy_value[j] <= 0.5){
				if(map->egma[k].grid[i].occupancy_value[j] <= nodeMinOccupancyValue){
					if((pow((this->randomPoint.first - i),2) + pow((this->randomPoint.second- j),2)) < (pow((this->randomPoint.first - best.first),2) + pow((this->randomPoint.second - best.second),2))){
						best = std::make_pair(i,j);
						nodeMinOccupancyValue = map->egma[k].grid[i].occupancy_value[j];
					}
				}
			}
		}
	}

	if(best.first == this->closest->x && best.second == this->closest->y){
		return;
	}
	//RCLCPP_WARN(this->get_logger(), "best state: x: %i, y: %i", best.first, best.second);

	TreeNode toAppend(best.first,best.second,k);
	if(toAppend.x == this->goal.x && toAppend.y == this->goal.y){
		this->iteration = 1000;
		toAppend.goal= true;
	}
	this->closest->children.push_back(toAppend);
	//RCLCPP_WARN(this->get_logger(), "children size: %i", (int)this->closest->children.size());
	//RCLCPP_WARN(this->get_logger(), "child: x: %i y: %i \n", this->closest->children[0].x, (int)this->closest->children[0].y);

	return;
}

void RRTNode::createPaths(Egma::SharedPtr map){
	srand( static_cast<unsigned int>(time(nullptr))) ;
	
	while(this->iteration < 1000){
		addNewRRTNode(map);
		iteration++;
	}
	
	return;
}

void RRTNode::bestPath(TreeNode *head, float total, std::vector< TreeNode > path, Egma::SharedPtr map){
	if(head == nullptr){
		if(total< this->tempPathCost){
			this->finalRRTPath = path;
			this->tempPathCost = total;
		}
		return;
	}
	
	std::vector<TreeNode> tempPath(path.begin(), path.end());
	tempPath.push_back(*head);

	if(head->x ==this->goal.x && head->y ==this->goal.y){
		this->finalRRTPath = tempPath;
		this->tempPathCost=-1;
		return;
	}

	total+=map->egma[head->index].grid[head->x].occupancy_value[head->y];

	for(int i =0; i< (int)head->children.size();i++){
		bestPath(&head->children[i], total, tempPath, map);
	}
	if(head->children.size() ==0){
		bestPath(nullptr, total, tempPath, map);
	}

	return;
} 

void RRTNode::findPath(Egma::SharedPtr map){
	/*while(this->goal.x == (-1)){
		RCLCPP_WARN(this->get_logger(), "in while: ");
	}*/

	this->goal.x = map->goal_point.x;
	this->goal.y = map->goal_point.y;
	this->goal.index = (-1);

	this->currentPosition = TreeNode(10,4, 0);
	this->closest = &currentPosition;

	this->currentMinCostForSingleNode = std::numeric_limits<float>::max();
	this->tempPathCost = std::numeric_limits<float>::max();

	this->iteration = 0;
	this->totalLeaves = 1;

	createPaths(map);
	std::vector<TreeNode> tempVec;
	bestPath(&this->currentPosition, 0, tempVec, map);

	//RrtPath msg;
	Path tempMsg;
	//RCLCPP_WARN(this->get_logger(), "size of path: %i", (int)this->finalRRTPath.size());
	tempMsg.header.frame_id = "map";

	for(int i=0; i< (int)this->finalRRTPath.size(); i++){
		Point pathPt;
		PoseStamped tempPose;
		tempPose.header.frame_id = "map";
		pathPt.x = finalRRTPath[i].x;
		pathPt.y = finalRRTPath[i].y;
		pathPt.z = 0;
		//msg.points.push_back(pathPt);
		tempPose.pose.position = pathPt;
		//RCLCPP_WARN(this->get_logger(), "point: %.i with x: %.6f y: %.6f z: ", i, path_pt.x, path_pt.y, path_pt.z);

		//msg.stamps.push_back(map->egma[finalRRTPath[i].index].header.stamp);
		tempPose.header.stamp = map->egma[finalRRTPath[i].index].header.stamp;
		//RCLCPP_WARN(this->get_logger(), "stamp: %i", map->egma[finalRRTPath[i].index].header.stamp.sec);

		if(i < (int)this->finalRRTPath.size() -1){
			float changeX = finalRRTPath[i].x - finalRRTPath[i+1].x;
			float changeY = finalRRTPath[i].y - finalRRTPath[i+1].y;
			float theta = atan(changeX/changeY);
			tempPose.pose.orientation.w = sin(theta/2);
			tempPose.pose.orientation.z = cos(theta/2);
			//msg.thetas.push_back(theta);
			//RCLCPP_WARN(this->get_logger(), "theta: %.6f", theta);

		}
		tempMsg.poses.push_back(tempPose);
	}

	//this->rrt_path_pub->publish(msg);
	this->rrt_path_publisher->publish(tempMsg);

	//this->goal.x = (-1);
	//this->goal.y = (-1);

}

