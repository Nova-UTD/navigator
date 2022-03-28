#pragma once

namespace Nova {
namespace BehaviorPlanner {

class BehaviorPlanner {

public:

    BehaviorPlanner();
    ~BehaviorPlanner();

    // checks for upcoming intersection for now
    bool upcoming_stop_sign();

    // for now, ticks until perception data ready to be consumed
    bool obstacles_present();

private:
    int tick = 0;

};


}
}