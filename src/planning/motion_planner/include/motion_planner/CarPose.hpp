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
            double vx;
            double vy;
			CarPose(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
			CarPose() : x(0), y(0), vx(0), vy(0) {}
		};
	}
}