#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <set>
#include <ctime>

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voltron_msgs/msg/final_path.hpp>

class WayPointRRT{
public:
	float gps_X;
	float gps_Y;
  	time_t current_time;
	WayPointRRT(float gps_X, float gps_Y, time_t current_time){
		this->gps_X = gps_X;
		this->gps_Y = gps_Y;
		this->current_time = current_time;
	}
};

class WayPointPath {
public:
	std::vector<WayPointRRT> path;
	WayPointPath(){
		return;
	}
	WayPointPath(std::vector<WayPointRRT> tempPath){
		path = tempPath;
	}
};


class Ogma {
	public:
		float og[11][9];
		time_t time;
		Ogma(){
			return;
		}
		Ogma(float temp_og[11][9]){
			for(int i =0; i<int(sizeof(temp_og));i++){
				for(int j =0; j<int(sizeof(temp_og[i])); j++){
					this->og[i][j] = temp_og[i][j];
				}
			}
			this->time = std::time(0);
		}
		int size(){
			return 11;
		}
};

class Dogma{
	public:
		Ogma dog[10];
		Dogma(){
			float temp_a[11][9] = {{(float)1.0,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1}};
			Ogma* temp = new Ogma(temp_a);
			this->dog[0] = *temp;
			this->dog[1] = *temp;
			this->dog[2] = *temp;
			this->dog[3] = *temp;
			this->dog[4] = *temp;
			this->dog[5] = *temp;
			this->dog[6] = *temp;
			this->dog[7] = *temp;
			this->dog[8] = *temp;
			this->dog[9] = *temp;
		}
};

class RRTNode : public rclcpp::Node {
public:
	RRTNode();
	WayPointPath path;
	WayPointPath* createTree(); //TODO: check for costMap input for parameter
};


//NOT DONE: TO COMPLETE: TO TEST:
//Code that I am continuing to work on in non ros environment to ensure the logic is correct first:

//
//  rrt.h
//  novatiral
//
/*
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <ctime>

class rrt_node{
    public:
        int x;
        int y;
        time_t t;
        std::vector< rrt_node > children;
        rrt_node (){

        }
        rrt_node(int x, int y, time_t t){
            this->x = x;
            this->y = y;
            this->t = t;
            //this->children = new std::vector<rrt_node>();
        }
};

class Ogma {
    public:
        std::vector<std::vector<float>> og;
        time_t time;
        Ogma(){
            return;
        }
        Ogma(std::vector<std::vector<float>> temp_og){
            this->og = std::vector< std::vector<float> >(11, std::vector<float>(8));
            for(int i =0; i<temp_og.size();i++){
                for(int j =0; j<temp_og[i].size(); j++){
                    this->og[i][j] = temp_og[i][j];
                }
            }
            this->time = std::time(0);
        }
        int size(){
            return 11;
        }
};

class Dogma{
    public:
        std::vector<Ogma> dog;
        Dogma(){
            std::vector<std::vector<float>> temp_a{{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1},{1,1,1,0,0,0,1,1,1}};
            Ogma *temp = new Ogma(temp_a);
            this->dog = std::vector< Ogma >(10);
            this->dog[0] = *temp;
            this->dog[1] = *temp;
            this->dog[2] = *temp;
            this->dog[3] = *temp;
            this->dog[4] = *temp;
            this->dog[5] = *temp;
            this->dog[6] = *temp;
            this->dog[7] = *temp;
            this->dog[8] = *temp;
            this->dog[9] = *temp;
        }
};
*/