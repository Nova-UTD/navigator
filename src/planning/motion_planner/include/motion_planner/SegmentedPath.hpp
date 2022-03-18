/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#pragma once

#include "motion_planner/PathPoint.hpp"
#include <vector>
#include <memory>
#include <functional>


//Unless you know that your input points are evenly spaced, make sure you use valid_points to make sure that all the input TrajectoryPoints are spaced properly! 
//The class assumes this to be the case in other methods.
namespace navigator
{
namespace MotionPlanner
{
class SegmentedPath
{
public:
	//sets spacing to be distance between first 2 points (0 if there are < 2 points)
	SegmentedPath(const std::shared_ptr<std::vector<PathPoint>> points) 
		: spacing(points->size() < 2 ? 0 : points->at(0).distance(points->at(1)))
		, points(points), cost(INFINITY) {}
	SegmentedPath(const std::shared_ptr<std::vector<PathPoint>> points, double spacing) : spacing(spacing), points(points), cost (INFINITY) {}
	//checks if points are spaced uniformly by spacing units apart.
	//allows slight tolerance for floating point weirdness (0.00000001 on the square distance between each point)
	//no argument method uses this path's points and spacing
	bool valid_points() const;
	bool valid_points(double spacing, const std::vector<PathPoint>& points) const;
	//creates a branch from an existing path
	//start_angle: the initial steering angle
	//end_angle: the target angle
	//steer_speed: radians/unit traveled. how quickly the angle changes from start to end angle.
	//size: the number of points to generate
	std::shared_ptr<std::vector<PathPoint>> create_branch(double steering_angle, double end_angle, double steer_speed, size_t size) const;
	//Evaluates func at every point along the path and returns the sum
	double sum(std::function<double(PathPoint)> func) const;
	//Returns the point along the path at a given arclength
	//throws exception if arclength is outside the bounds of the path [0,spacing*points.size()]
	PathPoint sample(double arclength) const;
	//Returns the distance to the closest point on the path (including on the segments between input points)
	//(if path is empty, returns -1)
	double distance(PathPoint p) const;
	//returns curvature of the path over the segment at the given index
	//returns the curvature at the closest valid point at index=0 and from the second to last point onward
	//	or infinity if the path isn't long enough
	//uses a "special" type of curvature, so it's not just 0 because we are using line segments
	//it's basically the magnitude of the difference in slopes in the parametric equations of the lines.
	double curvature(size_t index) const;
	//if index >= number of points, returns the heading of the last valid point
	//angle of the line formed by the point at index and index+1. measured in radians from [-pi,pi]
	double heading(size_t index) const;
	//returns the index of the closest point in points to p 
	size_t closest_point(PathPoint p) const;
	//Returns the arclength of the points of intersection of the line and the path
	//line is defined by x(t) = mx*t+x0, y(t) = my*t+y0
	std::vector<double> intersection(double mx, double my, double x0, double y0) const;
	const double spacing;
	const std::shared_ptr<std::vector<PathPoint>> points;
	double cost;
};
}
}

