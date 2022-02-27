#pragma once

namespace navigator
{
namespace MotionPlanner
{
class Collision
{
public:
    //s = arclength relative to the trajectory the collision occurs on
    //actual bounds of the collision
    double s_in, t_in, t_out;
    //padded bounds of collision (disallowing near-misses, enforcing following time)
    double t_safe_in, t_safe_out, s_safe_in;
    Collision(double s_in, double t_in, double t_out) 
        : s_in(s_in), t_in(t_in), t_out(t_out) {}
    Collision(double s_in, double t_in, double t_out, double t_safe_in, double t_safe_out, double s_safe_in) 
        : s_in(s_in), t_in(t_in), t_out(t_out), t_safe_in(t_safe_in), t_safe_out(t_safe_out), s_safe_in(s_safe_in) {}
};
}
}