#pragma once

namespace Nova {
namespace BehaviorPlanner {

class BehaviorPlanner {

public:

    BehaviorPlanner();
    ~BehaviorPlanner();

    bool upcoming_stop_sign();
    bool upcoming_yield_sign();
    bool upcoming_speedbump();
    bool destination_close();
    bool obstacles_present();
    bool immediate_collision();

private:

};


}
}