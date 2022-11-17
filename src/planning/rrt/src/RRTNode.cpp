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
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::ColorRGBA;
using voltron_msgs::msg::FinalPath;

//Mock occupancy grid

RRTNode::RRTNode() : Node("rrt_node"){
	//cost_map_sub = this -> create_subscriptison<>
	/*odom_sub = this->create_subscription<Odometry>("/odometry/filtered", 1, [this](Odometry::SharedPtr msg) {
		cached_odom = msg;
	});*/
	//path_pub = this->create_publisher<WayPointPath>("path_pub",1);
	
	// expect the cost map from  cost_map_sub to be passed through
	// expect current position to be passed from odom_sub
	this->path = createTree()->path;
}


WayPointPath* RRTNode::createTree(){
	//have set of states possible actions
	//use costmap to determine which state transisitoned would be the best
	// use heuristic to determine which path will be the best
	WayPointPath *temp = new WayPointPath();
	return temp;
}



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



float max_velocity = 20; 
class tempRRT{
	public:
		float max_velocity = 20; 
		float max_distance;
		Dogma* map;
		std::vector< std::unordered_set < std::pair<int,int> > > free_states;
		rrt_node* head;
		rrt_node* closest;
		float current_min;
		float conversion_factor;
		rrt_node* current_state;
		tempRRT(){
			this->map = new Dogma();
			this->max_distance = 2;
			this->current_min = std::numeric_limits<float>::max();
			this->head = new rrt_node(10,4,map->dog[0].time);
			this->closest = head;
			this->conversion_factor = 3;
			this-> current_state = head;
			for (int k =0; k< 10; k++){
				std::unordered_set < std::pair<int,int> > time_step_free;
				for (int i=0; i<map->dog[k].og.size();i++){
					for (int j=0; j<map[i].size();j++){
						if(map[i][j]<=0.5){
							std::pair<int, int> temp_pair(i, j);
							time_step_free.add(temp_pair);
						}
					}
				}
				this->free_states.push_back(time_step_free);
			}
		}

		std::pair<int,int> randomPair(){
			srand (time(NULL));
			this->free_states[rand() % length+1];
		}

		void find_closest_state(rrt_node* head, std::pair<int,int> random_point){
			if(head == NULL){
				return;
			}
			float temp_min = pow((head->x - random_point.first),2) + pow((head->y - random_point.second),2);
			if(temp_min < this->current_min){
				this->current_min = temp_min;
				this->closest = head;
			}
			for(int i=0; i<head->children.size();i++){
				find_closest_state(head->children[i], random_point);
			}
			return;
		}

		rrt_node* find_new_state_to_add(int k){
			std::pair<int,int> best(this->current_state->x, this->current_state->y);
			float max = 1;
			for (int i=this->closest->x; i<=this->conversion_factor; i++){
				for(int j =this->closest->y-this->conversion_factor; j<this->closest->y+this->conversion_factor; j++){
					std::pair<int, int> temp_pair(i,j);
					if (this->free_states[k].first.find(temp_pair) != this->free_states[k].first.end()){
						if(map[k][i][j] <= max){
							if((pow((this->closest->x - i),2) + pow((this->closest->j - j),2)) < (pow((this->closest->x - best.first),2) + pow((this->closest->j - best.second),2))){
								best = new std::pair<int, int>(i,j);
								max = map[k][i][j];
							}
						}
					}
				}
			}
			rrt_node* toReturn(i,j,map[k]->time);
			this->closest->children.push_back(toReturn);
			return toReturn;
			
		}


		std::vector< rrt_node* > create_path(rrt_node* head){
			std::vector< rrt_node* > path; 
			for (int k = 0; k< this->free_states.size(); k++){
				path.push_back(find_new_state_to_add(k));
			}
			return path;
		}

};



//NOT DONE: TO COMPLETE: TO TEST:
//Code that I am working on in non ros environment to ensure the logic is correct first:

//
//  main.cpp
//  novatiral
/*
#include <chrono>
#include <functional>
#include <memory>
#include <string>
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
#include <memory>
#include "./rrt.h"


class tempRRT{
    public:
        float max_velocity = 20;
        float max_distance;
        Dogma* map;
        std::vector< std::set < std::pair<int,int> > > free_states;
        rrt_node* head;
        rrt_node* closest;
        std::pair<int,int> random_point;
        float current_min;
        float conversion_factor;
        rrt_node* current_state;
        tempRRT(){
            this->map = new Dogma();
            this->max_distance = 2;
            this->current_min = std::numeric_limits<float>::max();
            this->head = new rrt_node(10,4,map->dog[0].time);
            this->closest = head;
            this->random_point = std::make_pair(-1, -1);
            this->conversion_factor = 3;
            this->current_state = head;
            for (int k =0; k< 10; k++){
                std::set < std::pair<int,int> > time_step_free;
                for (int i=0; i<map->dog[k].og.size();i++){
                    for (int j=0; j<map->dog[k].og[i].size();j++){
                        if(map->dog[k].og[i][j]<=0.5){
                            std::pair<int, int> temp_pair(i, j);
                            time_step_free.insert(temp_pair);
                        }
                    }
                }
                this->free_states.push_back(time_step_free);
            }
            
        }

        void randomPair(int k){
            srand( static_cast<unsigned int>(time(nullptr)));
            int i = rand() % this->free_states.size();
            for (std::set<std::pair<int, int>>::iterator itr = free_states[k].begin(); itr != free_states[k].end(); ++itr) {
                if(i==0){
                    this->random_point =  *itr;
                }
                i--;
            }
            return;
        }

        void find_closest_state(rrt_node* head, std::pair<int,int> random_point){
            if(head == NULL){
                return;
            }
            float temp_min = pow((head->x - random_point.first),2) + pow((head->y - random_point.second),2);
            if(temp_min < this->current_min){
                this->current_min = temp_min;
                this->closest = head;
            }
            for(int i=0; i<head->children.size();i++){
                find_closest_state(&head->children[i], random_point);
            }
            return;
        }

        rrt_node find_new_state_to_add(int k){
            std::pair<int,int> best(this->current_state->x, this->current_state->y);
            randomPair(k);
            find_closest_state(this->head, this->random_point);
            float max = 1;
            for (int i=this->closest->x; i<=this->conversion_factor; i++){
                for(int j =this->closest->y-this->conversion_factor; j<this->closest->y+this->conversion_factor; j++){
                    std::pair<int, int> temp_pair(i,j);
                    if (this->free_states[k].count(temp_pair)){
                        if(this->map->dog[k].og[i][j] <= max){
                            if((pow((this->random_point.first - i),2) + pow((this->random_point.second- j),2)) < (pow((this->random_point.first - best.first),2) + pow((this->random_point.second - best.second),2))){
                                best = std::make_pair(i,j);
                                max = map->dog[k].og[i][j];
                            }
                        }
                    }
                }
            }
            rrt_node toReturn(best.first,best.second,map->dog[k].time);
            this->closest->children.push_back(toReturn);
            return toReturn;
            
        }


        std::vector< rrt_node > create_path(){
            std::vector< rrt_node > path;
            for (int k = 0; k< this->free_states.size(); k++){
                path.push_back(find_new_state_to_add(k));
            }
            for(int i=0; i<path.size();i++){
                std::cout<<path[i].x<<" "<<path[i].y<<" "<<path[i].t<<std::endl;
            }
            return path;
        }

};





int main(int argc, const char * argv[]) {
    // insert code here...
    tempRRT rrt;
    rrt.create_path();
    return 0;
}

*/
