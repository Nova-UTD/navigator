/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuit.hpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


// 1) Update closest point through node update
// 2) Compute the lookahead point through node update
// 3) Compute the curvature
// 4) Finally, compute & publish steering angle

#include <chrono>
#include <memory>
#include <string>
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/trajectory_point.hpp"


namespace Nova {
namespace PurePursuit {

static constexpr double MIN_CURVATURE = 1.0 / 9e10;
static constexpr double WHEEL_BASE = 0.1;

class PurePursuit {

public:

    PurePursuit(float lookahead_distance);
    ~PurePursuit();

    double get_steering_angle(voltron_msgs::msg::Trajectory cur_trajectory);

private:

    // var
    voltron_msgs::msg::Trajectory trajectory;
    voltron_msgs::msg::TrajectoryPoint closest_point;
    voltron_msgs::msg::TrajectoryPoint lookahead_point;
    float lookahead_distance;
    double curvature;
    double steering_angle;

    // functions
    void set_lookahead_point();
    void compute_curvature();
    void compute_steering_angle();
    void set_lookahead_distance(float lookahead_distance);
    double compute_steering_effort();

};


}
}