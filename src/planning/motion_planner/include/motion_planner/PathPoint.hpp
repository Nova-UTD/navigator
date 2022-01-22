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
			double x;
			double y;
			PathPoint(double x, double y) : x(x), y(y) {}
			PathPoint() : x(0), y(0) {}
			double distance(const PathPoint &other) const {
				return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
			}
		};
	}
}