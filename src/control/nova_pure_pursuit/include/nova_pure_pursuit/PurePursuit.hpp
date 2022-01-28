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


namespace Nova {
namespace PurePursuit {

class PurePursuit {

public:
    PurePursuit(float lookahead_distance);
    ~PurePursuit();

    
    void set_lookahead_point(float x, float y);
    void set_closest_point(float x, float y);
    float compute_steering_angle();

    //Test Node interface
    std::string hello_world();

private:

    float closest_x;
    float closest_y;
    
    float lookahead_distance;
    float lookahead_x;
    float lookahead_y;

    float curvature;
    float steering_angle;
    


}; // class PurePursuit


} // namespace PurePursuit
} // namespace Nova