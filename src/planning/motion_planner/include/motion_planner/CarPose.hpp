/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "autoware_auto_msgs/msg/detail/complex32__struct.hpp"

//temporary class for input from behavior planner
namespace navigator
{
	namespace MotionPlanner
	{
		class CarPose
		{
		public:
			double x;
			double y;
			autoware_auto_msgs::msg::Complex32 heading;
            double longitudinal_v;
            double lateral_v;
			CarPose(double x, double y, autoware_auto_msgs::msg::Complex32 heading, double longitudinal_v, double lateral_v) : x(x), y(y), heading(heading), longitudinal_v(longitudinal_v), lateral_v(lateral_v) {}
			CarPose() : x(0), y(0), longitudinal_v(0), lateral_v(0) {
				heading.real = 0;
				heading.imag = 0;
			}
		};
	}
}