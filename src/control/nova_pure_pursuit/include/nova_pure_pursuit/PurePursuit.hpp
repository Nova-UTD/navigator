/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuit.hpp
 * Author:    Cristian Cruz
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


namespace Nova {
namespace PurePursuit {

static constexpr double MIN_CURVATURE = 1.0 / 9e10;

class PurePursuit {

public:
    PurePursuit(float lookahead_distance);
    ~PurePursuit();

    double get_steering_angle(voltron_msgs::msg::Trajectory trajectory);
    void compute_curvature();
    void compute_steering_angle();
    void set_lookahead_point(float x, float y);
    void set_closest_point(float x, float y);


private:

    float closest_x;
    float closest_y;
    
    float lookahead_distance;
    float lookahead_x;
    float lookahead_y;

    double curvature;
    double steering_angle;


}; // class PurePursuit


} // namespace PurePursuit
} // namespace Nova