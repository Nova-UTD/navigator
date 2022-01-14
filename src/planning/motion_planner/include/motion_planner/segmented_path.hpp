/*
 * Package:   motion_planner
 * Filename:  motion_planner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */
#pragma once

#include "motion_planner/path_point.hpp"
#include <vector>
#include <memory>
#include <functional>


//Unless you know that your input points are evenly spaced, make sure you use valid_points to make sure that all the input TrajectoryPoints are spaced properly! 
//The class assumes this to be the case in other methods.
namespace navigator
{
namespace motion_planner
{
class segmented_path
{
public:
	//sets spacing to be distance between first 2 points (0 if there are < 2 points)
	segmented_path(const std::shared_ptr<const std::vector<path_point>> points) 
		: spacing(points->size() < 2 ? 0 : points->at(0).distance(points->at(1)))
		, points(points) {}
	segmented_path(const std::shared_ptr<const std::vector<path_point>> points, double spacing) : spacing(spacing), points(points) {}
	//checks if points are spaced uniformly by spacing units apart.
	//allows slight tolerance for floating point weirdness (0.00000001 on the square distance between each point)
	//no argument method uses this path's points and spacing
	bool valid_points() const;
	bool valid_points(double spacing, const std::vector<path_point>& points) const;
	//creates a branch from an existing path
	//start_angle: the initial steering angle
	//end_angle: the target angle
	//steer_speed: radians/unit traveled. how quickly the angle changes from start to end angle.
	//size: the number of points to generate
	std::shared_ptr<std::vector<path_point>> create_branch(double steering_angle, double end_angle, double steer_speed, size_t size) const;
	//Evaluates func at every point along the path and returns the sum
	double sum(std::function<double(path_point)> func) const;
	//Returns the point along the path at a given arclength
	//throws exception if arclength is outside the bounds of the path [0,spacing*points.size()]
	path_point sample(double arclength) const;
	//Returns the distance to the closest point on the path (including on the segments between input points)
	//(if path is empty, returns -1)
	double distance(path_point p) const;
	const double spacing;
	const std::shared_ptr<const std::vector<path_point>> points;
};
}
}

