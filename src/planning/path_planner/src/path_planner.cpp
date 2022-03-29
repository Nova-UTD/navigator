/*
 * Package:   path_planner
 * Filename:  path_panner.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

/* Todo list for this file (roughly sorted most important first)
 *  - Implement safety cost (obstacles etc)
 *  - better guess logic for current lanelet (use predicted path)
 *  - Check implementation for short/choppy lanelets
 *  - Better horizon (see .hpp)
 *  - Special non-cruising paths like pull over
 *  - Account for horizontal lane change distance in lanelet sequence generation
 *  - Move visualization to a different node (PathPlannerNode)
 *
 *  - Documentation
 *  - Copied a lot of code from Lanelet2GlobalPlanner and I think there's a better
 *    way to reuse it. Applies to PathPlannerNode as well.
 *  - Copyright info
 *
 */
#include <path_planner/path_planner.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <voltron_msgs/msg/route_costs.hpp>
#include <voltron_msgs/msg/route_cost.hpp>
#include <voltron_msgs/msg/costed_path.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <boost/geometry/geometries/segment.hpp>

#include <string.h>

using namespace navigator::path_planner;

using voltron_msgs::msg::CostedPath;
using voltron_msgs::msg::RouteCost;
using voltron_msgs::msg::RouteCosts;

PathPlanner::PathPlanner()
{
    // Todo: what needs to be initialized?
}

////////////////////////////////////// Main logic /////////////////////////////////////

/**
 * @brief Generates all paths representing an option for vehicle motion.
 *
 * Paths should represent the ideal case for the set of paths they represent:
 * for instance, when following a given lane, there are many space paths the
 * vehicle could travel in but the ideal representative path for this set
 * would follow the centerline.
 *
 * @param current_pose
 * @param route_costs
 * @param results - paths are written back to this vector
 */
void PathPlanner::get_paths(
    const geometry_msgs::msg::Pose &current_pose,
    const std::vector<RouteCost> &route_costs,
    std::vector<CostedPath> &results)
{
    // Update current state
    set_routing_costs(route_costs);
    set_pose(current_pose);

    // Grab starting location and use it to generate all lanelet-level paths
    auto current_lanelet = get_current_lanelet();
    double horizon = get_horizon();

    // Generate lanelet-level path options
    std::vector<LaneletSequence> lanelet_sequences;
    get_lanelet_sequences(horizon, current_pose.position, current_lanelet, lanelet_sequences);

    // Convert lanelet-level paths to space paths
    for (auto sequence : lanelet_sequences)
    {
        results.push_back(get_path(sequence, current_pose));
    }

    // TODO: consider adding special curves like pull over
}

/**
 * @brief Generate the space-curve following the lanelet sequence
 *
 * @param lanelets
 * @return CostedPath
 */
CostedPath PathPlanner::get_path(const LaneletSequence &lanelet_sequence, const geometry_msgs::msg::Pose& pose)
{
    CostedPath path;

    if (lanelet_sequence.size() == 0)
    {
        return path;
    }

    // Somewhat naive lane change handling: drop all the but last lanelet 
    // in a string of adjacent lanelets

    // Lanelets to visit, after removing pre-lane-change lanelets
    LaneletSequence lanelets;

    // loop through and add all the lanelets we want to lanelets.
    // worse in space but better in time than deleting from lanelet_sequence
    for (auto id_it = lanelet_sequence.begin(); id_it != lanelet_sequence.end(); id_it++)
    {
        if ((id_it + 1) == lanelet_sequence.end() || !is_adjacent(*(id_it), *(id_it + 1)))
        {
            // either the end of or not part of a string of adjacent lanelets,
            // add to post-filter sequence
            lanelets.push_back((*id_it));
        }
    }

    // start at current position
    path.points.push_back(pose.position);

    for (auto lanelet_id_it = lanelets.begin(); lanelet_id_it != lanelets.end(); lanelet_id_it++)
    {
        lanelet::Lanelet lanelet = map->laneletLayer.get(*lanelet_id_it);

        // Connect to the path
        // Split based on the last path point and only take the part of
        // the lanelet centerline to that needs to be traveled
        // lanelet-compatible point for current position
        lanelet::BasicPoint2d l_point = PathPlanner::ros_point_to_lanelet_point(path.points.back());

        // Splitting by current position means we can travel only the half we need to
        lanelet::BasicLineString2d dont_need_to_travel;
        lanelet::BasicLineString2d need_to_travel;
        split_linestring(lanelet.centerline2d(), l_point, dont_need_to_travel, need_to_travel);

        for (auto seg_point : need_to_travel)
        {
            path.points.push_back(PathPlanner::lanelet_point_to_ros_point(seg_point));
        }
    }

    // Set route cost of path to route cost of last lanelet
    // TODO: what about paths that include but go past the destination?
    path.routing_cost = get_routing_cost(*(lanelets.end() - 1));

    return path;
}

////////////////////////////////////// Lanelet-level generation /////////////////////////////////////

void PathPlanner::get_lanelet_sequences(double distance, const geometry_msgs::msg::Point starting_point,
                                        lanelet::ConstLanelet starting_lanelet,
                                        std::vector<LaneletSequence> &results)
{

    // Recalculate remaining distance based on position in current lanelet
    lanelet::BasicPoint2d position = PathPlanner::ros_point_to_lanelet_point(starting_point);
    auto centerline = starting_lanelet.centerline2d();
    lanelet::BasicLineString2d first_half;
    lanelet::BasicLineString2d second_half;
    PathPlanner::split_linestring(centerline, position, first_half, second_half);
    
    // We will be substracting whole length of lanelet, resulting in a net decrease only
    // of the distance we actually travel
    distance += static_cast<double>(lanelet::geometry::length(first_half));

    _get_lanelet_sequences(distance, LaneletSequence(), starting_lanelet, results);
}

/**
 * @brief Recursively generate a list of lanelets that paths lead through and add them
 * to results.
 *
 * Continues recursion until there is no remaining distance to cover. Since lanelets may
 * have multiple successors/adjacents, each call may branch into multiple so the sequence
 * must be passed-by-value.
 *
 * Lanelets can make multiple adjacent changes before moving to the next succeeding change,
 * but they must be in the same direction (i.e. Left-Left or Right-Right but not Left-Right)
 *
 * remaining_distance is distance BEFORE current is added.
 *
 * @param remaining_distance - remaining length of lanelets to generate the sequence for
 * @param previous  - List of all previous lanelet ids in the sequence.
 * @param current - Lanelet being added to the sequence and recursed for
 * @param results  - writeback destination of sequences
 */
void PathPlanner::_get_lanelet_sequences(
    double remaining_distance, LaneletSequence sequence, lanelet::ConstLanelet current, std::vector<LaneletSequence> &results)
{
    if (remaining_distance <= 0)
    {
        // Have no more distance to travel and this sequence is complete
        return;
    }
    // Checking distance at the beginning of the function rather than after adding the
    // current lanelet results in more unecessary loops of checking next lanelets, but
    // lets us account for adjacencies in lanes that we could reach but not fully follow

    // add current lanelet to the sequence
    sequence.push_back(current.id());

    // TODO: this is a temporary measure to patch issues like edge-of-map or
    // proceeding past the destination, but the full implications of paths that 
    // stop partway through have not been considered yet.         
    results.push_back(sequence);

    double current_length = lanelet::geometry::length2d(current);
    remaining_distance -= current_length;

    // still need to cover some distance, look for more lanelets

    // Must assure that we don't return to the previous lane
    // Use current id as a placeholder until we're sure sequence has a previous
    lanelet::Id previous_id = current_lanelet.id();
    lanelet::ConstLanelet previous_lanelet = map->laneletLayer.get(previous_id);
    if (sequence.size() > 1)
    {
        // previous is located two from the end since we added current
        previous_id = *(sequence.end() - 2);
    }

    // Successors and adjacents both included in following(~, true)
    auto relations = routing_graph->following(current, true);

    for (lanelet::ConstLanelet next : relations)
    {
        if (next.id() != previous_id)
        {
            double distance = remaining_distance;
            if (is_adjacent(previous_lanelet, next))
            {
                // If we are moving to an adjacent lanelet, then we are not
                // using the full length of either lane. Adjust distance remaining.
                distance += current_length;
            }
            _get_lanelet_sequences(distance, sequence, next, results);
        }
    }
}

////////////////////////////////////// State setters /////////////////////////////////////

/**
 * @brief Sets up map-related resources for this class
 *
 * @param map
 */
void PathPlanner::set_map(lanelet::LaneletMapPtr map)
{
    this->map = map;

    // generate routing graph
    routing_graph.release();
    lanelet::traffic_rules::TrafficRulesPtr trafficRules =
        lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, // TODO: change locations
            lanelet::Participants::Vehicle);
    routing_graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);
}

/**
 * @brief Updates routing costs to new values. Erases old costs
 *
 * @param costs
 */
void PathPlanner::set_routing_costs(const std::vector<voltron_msgs::msg::RouteCost> &costs)
{
    // wipe previous route costs and update with new values
    route_cost_map.clear();

    for (const RouteCost cost : costs)
    {
        route_cost_map[lanelet::Id(cost.lane_id)] = cost.cost;
    }
}

// helper for set_pose
enum LaneletPriority : int
{
    // Numbers are negative so that CURRENT > EXPECTED > CONTAINING > NEARBY
    // but more levels can be added below
    CURRENT = -1,
    EXPECTED = -2,
    CONTAINING = -3,
    NEARBY = -4,

    // placeholder for no priority yet
    NOT_SET = -100
};

/**
 * @brief Updates planner to current position
 *
 * Determines lanelet from position and orientation. Priority list:
 * 1. (Highest Priority) If we have not exited the current lanelet, keep it
 * 2. Containing lanelets we expected to be in by following the selected path
 * 3. Containing lanelets that best match position and orientation
 * 4. Nearby lanelets that best match position and orientation
 */
void PathPlanner::set_pose(geometry_msgs::msg::Pose pose)
{

    const lanelet::BasicPoint2d lanelet_position = PathPlanner::ros_point_to_lanelet_point(pose.position);

    // Check if current lanelet still applies
    if (lanelet::geometry::inside(current_lanelet, lanelet_position))
    {
        // retain current lanelet
        return;
    }

    // Fetch nearby lanelets
    // Todo: Made up 5 on the spot. Find a better number like max conflicts in map
    lanelet::Lanelets lanelets = map->laneletLayer.nearest(lanelet_position, 5);

    // Linear search for best lanelet
    // Order by higher priority and then by higher pose match
    lanelet::Lanelet best_match_lanelet;
    LaneletPriority priority = LaneletPriority::NOT_SET;
    double pose_match = std::numeric_limits<double>::min();

    for (auto lanelet : lanelets)
    {
        LaneletPriority curr_priority = LaneletPriority::NOT_SET;
        double curr_pose_match = -std::numeric_limits<double>::max();

        if (lanelet::geometry::inside(lanelet, lanelet_position))
        {
            // TODO: check predicted
            curr_priority = LaneletPriority::CONTAINING;
        }
        else
        {
            curr_priority = LaneletPriority::NEARBY;
        }

        // Calculate pose metric.
        // TODO: incorperate orientation. Distance only for now.
        curr_pose_match = -lanelet::geometry::distanceToCenterline2d(lanelet, lanelet_position);

        // Higher priority automatically wins. Equal priority orders by pose match
        if ((curr_priority > priority) || ((curr_priority == priority) && (curr_pose_match > pose_match)))
        {
            // have new best lanelet
            best_match_lanelet = lanelet;
            priority = curr_priority;
            pose_match = curr_pose_match;
        }
    }

    // Update current with new best guess
    current_lanelet = best_match_lanelet;
}

////////////////////////////////////////// Helper Methods /////////////////////////////////////////

lanelet::Lanelet PathPlanner::get_current_lanelet()
{
    return current_lanelet;
}

/**
 * @brief Returns routing cost of path, or max double if none is found
 *
 * @param id
 * @return double
 */
double PathPlanner::get_routing_cost(lanelet::Id id)
{
    if (route_cost_map.find(id) != route_cost_map.end())
    {
        return route_cost_map[id];
    }
    else
    {
        return std::numeric_limits<double>::max();
    }
}

/**
 * @brief Converts a point message to a lanelet-compatable point2d
 *
 * @param point
 * @return lanelet::BasicPoint2d
 */
lanelet::BasicPoint2d PathPlanner::ros_point_to_lanelet_point(geometry_msgs::msg::Point point)
{
    return lanelet::BasicPoint2d(point.x, point.y);
}

geometry_msgs::msg::Point PathPlanner::lanelet_point_to_ros_point(lanelet::BasicPoint2d point)
{
    geometry_msgs::msg::Point ros_point;
    ros_point.set__x(point.x());
    ros_point.set__y(point.y());
    return ros_point;
}

/**
 * @brief Checks if the two lanelets are related via a left or right relation
 *
 * @param l1
 * @param l2
 * @return true if l1 and l2 are adjacent
 * @return false if l1 or l2 are non-adjacent
 */
bool PathPlanner::is_adjacent(const lanelet::ConstLanelet l1, const lanelet::ConstLanelet l2)
{
    auto relation = routing_graph->routingRelation(l1, l2);

    if (!relation)
    {
        // lanelets are not related and therefore not adjacent
        return false;
    }

    lanelet::routing::RelationType r = relation.get();
    return (r == lanelet::routing::RelationType::AdjacentLeft 
            || r == lanelet::routing::RelationType::AdjacentRight
            || r == lanelet::routing::RelationType::Left
            || r == lanelet::routing::RelationType::Right);
}

/**
 * @brief Checks if the two lanelets are immediately related via a left or right relation
 *
 * @param l1
 * @param l2
 * @return true if l1 and l2 are adjacent
 * @return false if l1 or l2 are non-adjacent
 */
bool PathPlanner::is_adjacent(const lanelet::Id li1, const lanelet::Id li2){
    return is_adjacent(map->laneletLayer.get(li1), map->laneletLayer.get(li2));
}

/**
 * @brief Returns if two points are equal.
 *
 * Not particularily robust. Using a tolerance that works
 * for the scope of the path planner, i.e. point measurements
 * are in meters and we don't care about accuracy past ~1cm
 *
 * @param p1
 * @param p2
 * @return true
 * @return false
 */
bool point_tolerant_equals(const lanelet::BasicPoint2d &p1, const lanelet::BasicPoint2d &p2)
{
    return (abs(p1.x() - p2.x()) < 0.001) && (abs(p1.y() - p2.y()) < 0.001);
}

/**
 * @brief Splits a non-empty linestring at the location of a point projected onto the linestring.
 *
 * @param original - Linestring to split
 * @param split_point - Split location
 * @param first_half - Linestring before split
 * @param second_half - Linestring after split
 */
void PathPlanner::split_linestring(const lanelet::ConstLineString2d original, lanelet::BasicPoint2d split_point, lanelet::BasicLineString2d &first, lanelet::BasicLineString2d &second)
{
    // project the point into the linestring
    lanelet::BasicPoint2d point = lanelet::geometry::project(original, split_point);

    // Strategy: iterate through linestring until the projected point lies on a segment. Split at that segment.

    // Iterator to first vertice of line segment
    auto it_seg_p1 = original.begin();

    if (point_tolerant_equals((*it_seg_p1).basicPoint2d(), point))
    {
        // projected onto start, so split puts whole linestring into second_half
        second = original.basicLineString();
        return;
    }

    // Iterator to second vertice of line segment
    auto it_seg_p2 = it_seg_p1 + 1;

    // Continue until we run out of linestring, the point lies on a vertice, or the point lies between a vertice
    while (it_seg_p2 != original.end())
    {
        // Check vertice equivalence
        if (point_tolerant_equals((*it_seg_p1).basicPoint2d(), point))
        {
            // Split at seg_p2
            first.push_back(it_seg_p1->basicPoint2d());
            first.push_back(it_seg_p2->basicPoint2d());

            // add rest of linestring to second half
            while (it_seg_p2 != original.end())
            {
                second.push_back(it_seg_p2->basicPoint2d());
                it_seg_p2++;
            }
            return;
        }

        // Check if between vertices, again using very high tolerance
        boost::geometry::model::segment<lanelet::BasicPoint2d> segment(*it_seg_p1, *it_seg_p2);
        if (boost::geometry::distance(segment, point) < 0.001)
        {
            // Break at point itself.
            first.push_back(it_seg_p1->basicPoint2d());
            first.push_back(point);

            second.push_back(point);
            while (it_seg_p2 != original.end())
            {
                second.push_back(it_seg_p2->basicPoint2d());
                it_seg_p2++;
            }
            return;
        }

        // Move to next segment
        first.push_back(it_seg_p1->basicPoint2d());

        it_seg_p2++;
        it_seg_p1++;
    }

    // If no break was found, first will contain the original line string
}
