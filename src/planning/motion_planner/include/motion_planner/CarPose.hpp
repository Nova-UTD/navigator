/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

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
			double heading;
            double xv;
            double yv;
			double width, length;
			CarPose(double x, double y, double heading, double xv, double yv, double width, double length)
				: x(x), y(y), heading(heading), xv(xv), yv(yv), width(width), length(length) {}
			CarPose(double x, double y, double heading, double xv, double yv) : x(x), y(y), heading(heading), xv(xv), yv(yv), width(0), length(0) {}
			CarPose() : x(0), y(0), heading(0), xv(0), yv(0), width(0), length(0) {}
		};
	}
}