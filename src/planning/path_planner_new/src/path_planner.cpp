#include <path_planner_new/path_planner.hpp>

#include <voltron_msgs/msg/route_cost.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

using namespace navigator::path_planner;

using voltron_msgs::msg::RouteCost;

PathPlanner::PathPlanner()
{
}

void PathPlanner::set_map(lanelet::LaneletMapPtr map)
{
    std::cout << map; // compiler is yelling at me for unused parameters 
}

void PathPlanner::set_routing_costs(std::vector<RouteCost> costs)
{
    for(auto cost : costs){
        std::cout << "Lane ID: " << cost.lane_id << " Cost: " << cost.cost;
    }
}

// designed for recursion
void PathPlanner::get_lanelet_sequences(double remaining_distance,
                                        LaneletSequence previous, std::vector<LaneletSequence> &results)
{
    if(previous.empty() || remaining_distance || results.empty()){
        std::cout << "c";
    }
}

// Todo: return types
void PathPlanner::getCruisePaths() {}  // for normal driving: drive forward and lane change
void PathPlanner::getSpecialPaths() {} // for special paths like pull over, maybe not needed
