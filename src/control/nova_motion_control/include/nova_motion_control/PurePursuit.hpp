/*
 * Package:   nova_motion_control
 * Filename:  PurePursuit.hpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

namespace Nova {
namespace PurePursuit {

static constexpr double MIN_CURVATURE = 1.0 / 9e10;
static constexpr double WHEEL_BASE = 3.39; // meters

class PurePursuit {

public:

    PurePursuit(float lookahead_distance);
    ~PurePursuit();

    double get_steering_angle();
    float get_lookahead_distance();

    float get_lookahead_point_x();
    float get_lookahead_point_y();

    float get_displacement_error();

    void set_lookahead_point(float x, float y);
    void set_displacement_error(float displacement);

    float compute_steering_angle();
    float compute_curvature();

private:

    // var
    float closest_point_x;
    float closest_point_y;

    float lookahead_point_x;
    float lookahead_point_y;

    float lookahead_distance;
    double curvature;
    double steering_angle;
    float displacement_error;

    // functions
    void set_lookahead_distance(float lookahead_distance);
    double compute_steering_effort();

};


}
}