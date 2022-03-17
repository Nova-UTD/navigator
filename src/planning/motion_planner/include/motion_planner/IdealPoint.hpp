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
		class IdealPoint
		{
		public:
			double x;
			double y;
            double speed;
			IdealPoint(double x, double y, double speed) : x(x), y(y), speed(speed) {}
			IdealPoint() : x(0), y(0), speed(0) {}
		};
	}
}