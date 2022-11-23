
#include "nova_behavior_planner/BehaviorPlanner.hpp"

using namespace Nova::BehaviorPlanner;

BehaviorPlanner::BehaviorPlanner() {}

BehaviorPlanner::~BehaviorPlanner() {}


// od
bool BehaviorPlanner::upcoming_stop_sign() {

    bool conditionTrue = false;
    return conditionTrue;

}


// percp
bool BehaviorPlanner::obstacles_present() {
    // bool conditionTrue = false;
    // return conditionTrue;

    // tick
    if (tick == 3) {
        tick = 0;
        return false;    
    } else {
        tick++;
        return true;
    }
}
