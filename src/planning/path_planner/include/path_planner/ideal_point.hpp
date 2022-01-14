#pragma once

//temporary class for input from behavior planner
namespace navigator
{
	namespace path_planner
	{
		class ideal_point
		{
		public:
			double x;
			double y;
            double speed;
			ideal_point(double x, double y, double speed) : x(x), y(y), speed(speed) {}
			ideal_point() : x(0), y(0), speed(0) {}
		};
	}
}