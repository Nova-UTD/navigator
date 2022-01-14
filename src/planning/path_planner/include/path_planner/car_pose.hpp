#pragma once

//temporary class for input from behavior planner
namespace navigator
{
	namespace path_planner
	{
		class car_pose
		{
		public:
			double x;
			double y;
            double vx;
            double vy;
			car_pose(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
			car_pose() : x(0), y(0), vx(0), vy(0) {}
		};
	}
}