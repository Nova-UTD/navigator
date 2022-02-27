/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "motion_planner/SegmentedPath.hpp"
#include <stdlib.h>
#include <stdexcept>
#include <string>
#include <memory>
#include <math.h>

using std::vector;
using std::sin;
using std::cos;
using std::floor;
using std::sqrt;
using std::atan2;

using namespace navigator::MotionPlanner;

//checks if this path's points are spaced uniformly by spacing units apart.
//allows slight tolerance for floating point weirdness (0.00000001 on the square distance between each point)
bool SegmentedPath::valid_points() const {
	return valid_points(spacing, *points);
}
//checks if points are spaced uniformly by spacing units apart.
//allows slight tolerance for floating point weirdness (0.00000001 on the square distance between each point)
bool SegmentedPath::valid_points(double spacing, const vector<PathPoint>& points) const
{
	const double TOLERANCE = 0.00000001;
	if (spacing < 0) return false;
	double sqr_spacing = spacing * spacing;
	for (size_t i = 0; i < points.size() - 1; i++) {
		double sqr_dist = (points[i].x - points[i + 1].x) * (points[i].x - points[i + 1].x) + (points[i].y - points[i + 1].y) * (points[i].y - points[i + 1].y);
		if (abs(sqr_spacing - sqr_dist) > TOLERANCE) return false; //check if points are not approximately spacing units apart
	}
	return true;
}

//creates a branch from an existing path
//start_angle: the initial angle
//max_change: the maximum amount the angle can change
//steer_speed: radians/unit traveled. how quickly the angle changes from start to end angle.
//size: the number of points to generate
std::shared_ptr<std::vector<PathPoint>> SegmentedPath::create_branch(double start_angle, double max_change, double steer_speed, size_t size) const
{
	double current_angle = start_angle;
	const double min_angle = (max_change > 0) ? start_angle-max_change : start_angle+max_change;
	const double max_angle = (max_change > 0) ? start_angle+max_change : start_angle-max_change;
	PathPoint current = points->front();
	std::shared_ptr<vector<PathPoint>> result = std::make_shared<vector<PathPoint>>();
	for (size_t i = 0; i < size; i++) {
		double dx = cos(current_angle) * spacing;
		double dy = sin(current_angle) * spacing;
		PathPoint p(current.x + dx, current.y + dy);
		result->push_back(PathPoint(current.x + dx, current.y + dy));
		
		current = p;
		current_angle += steer_speed * spacing;
		//bound by max_change
		if (current_angle < min_angle) current_angle = min_angle;
		if (current_angle > max_angle) current_angle = max_angle;
	}
	return result;
}

double SegmentedPath::sum(std::function<double(PathPoint)> func) const
{
	double sum = 0;
	for (const auto& point : *points) {
		sum += func(point);
	}
	return sum;
}

PathPoint SegmentedPath::sample(double arclength) const
{
	//first check if arclength is outside the curve's bounds
	if (arclength < 0 || arclength > spacing * static_cast<double>(points->size())) {
		throw std::domain_error("Arc legnth (" + std::to_string(arclength) + ") is out of range of the curve.");
	}
	double segment = floor(arclength / spacing);
	
	size_t segment_id = static_cast<size_t>(segment); //index of the first point
	PathPoint a = points->at(segment_id);
	PathPoint b = points->at(segment_id+1);
	//we assume a and b are spacing units apart (can check with valid_points)
	double progress = (arclength - segment) / spacing; //proportion between a and b the target point is
	return PathPoint(a.x + (b.x - a.x) * progress, a.y + (b.y - a.y) * progress);
}

//probably should optimize this later. currently does a linear scan of all segments
double SegmentedPath::distance(PathPoint p) const
{
	//-1 gets returned if path is empty
	double min = -1;
	//using approach from https://monkeyproofsolutions.nl/wordpress/how-to-calculate-the-shortest-distance-between-a-point-and-a-line/
	for (size_t i = 0; i < points->size() - 1; i++) {
		
		PathPoint a = points->at(i);
		PathPoint b = points->at(i + 1);
		double m_x = b.x - a.x;
		double m_y = b.y - a.y;
		double da_x = p.x - a.x;
		double da_y = p.y - a.y;
		double t = (da_x * m_x + da_y * m_y) / (m_x * m_x + m_y * m_y);
		if (t < 0) {
			//closest point is a
			double d = sqrt(da_x * da_x + da_y * da_y);
			if (d < min || min == -1) {
				min = d;
			}
		}
		else if (t > 1) {
			//closest point is b
			double d = sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));
			if (d < min|| min == -1) {
				min = d;
			}
		}
		else {
			//closest point is on the segment
			double progress_x = a.x + t * m_x;
			double progress_y = a.y + t * m_y;
			double d = sqrt((p.x - progress_x) * (p.x - progress_x) + (p.y - progress_y) * (p.y - progress_y));
			if (d < min|| min == -1) {
				min = d;
			}
		}
	}

	return min;
}

size_t SegmentedPath::closest_point(PathPoint p) const {
	double min_dist = INFINITY;
	size_t min_index = -1;
	for (size_t i = 0; i < points->size(); i++) {
		double d = (p.x - points->at(i).x) * (p.x - points->at(i).x) 
			+ (p.y - points->at(i).y) * (p.y - points->at(i).y);
		if (d < min_dist) {
			min_index = i;
			min_dist = d;
		}
	}
	return min_index;
}

std::vector<double> SegmentedPath::intersection(double mx, double my, double x0, double y0) const {
	if (mx == 0 && my == 0) {
		//TODO: return arclength is x0,y0 is on (or close enough) to line
		return std::vector<double>();
	}
	double slope = mx/my;
	bool use_x_slope = false;
	if (my == 0) {
		//we need to use my/mx slope instead of mx/my to avoid division by 0
		use_x_slope = true;
		slope = my/mx;
	}
	
	//for each line segment:
	//1. calculate intersection of given line and current segment
	//2. if the intersection is outisde the bounds of the segment, move on to next segment
	//3. if it is within the boundary, calculate the arclength of the intersection relative to this segment.
	//4. then add i*spacing to that, push the value onto the vector, and move on to next segment
	std::vector<double> intersections;
	for (int i = 0; i < points->size()-1; i++) {
		auto a = points->at(i);
		auto b = points->at(i+1);
		double proportion;
		if (use_x_slope) {
			proportion = (a.y-y0-a.x+slope*x0)/(slope*(b.x-a.x)+a.y-b.y);	
		} else {
			proportion = (a.x-x0-a.y+slope*y0)/(slope*(b.y-a.y)+a.x-b.x);
		}
		if (0 <= proportion && proportion < 1) {
			//intersection is between a and b
			double arclength = i*spacing + proportion*spacing;
			intersections.push_back(arclength);
		}
	}
	return intersections;
}
