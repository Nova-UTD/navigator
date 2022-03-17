/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <math.h>
namespace navigator
{
	namespace MotionPlanner
	{
		class PathPoint
		{
		public:
			double x,y,vx,vy;
			PathPoint(double x, double y) : x(x), y(y), vx(0), vy(0) {}
			PathPoint(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
			PathPoint() : x(0), y(0), vx(0), vy(0) {}
			double distance(const PathPoint &other) const {
				return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
			}
		};
	}
}