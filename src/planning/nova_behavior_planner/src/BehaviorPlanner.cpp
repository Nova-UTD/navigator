
#include "nova_behavior_planner/BehaviorPlanner.hpp"

using namespace Nova::BehaviorPlanner;

BehaviorPlanner::BehaviorPlanner() {}

BehaviorPlanner::~BehaviorPlanner() {}


// od
bool BehaviorPlanner::upcoming_stop_sign() {

    bool conditionTrue = false;
    return conditionTrue;

}

// od
bool BehaviorPlanner::upcoming_yield_sign() {
    bool conditionTrue = false;
    return conditionTrue;
}

// od
bool BehaviorPlanner::upcoming_speedbump() {
    bool conditionTrue = false;
    return conditionTrue;
}


bool BehaviorPlanner::destination_close() {
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

// percp
bool BehaviorPlanner::immediate_collision() {
    bool conditionTrue = false;
    return conditionTrue;
}