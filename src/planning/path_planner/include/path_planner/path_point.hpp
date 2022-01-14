#pragma once

#include <math.h>
namespace navigator
{
	namespace path_planner
	{
		class path_point
		{
		public:
			double x;
			double y;
			path_point(double x, double y) : x(x), y(y) {}
			path_point() : x(0), y(0) {}
			double distance(const path_point &other) const {
				return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
			}
		};
	}
}