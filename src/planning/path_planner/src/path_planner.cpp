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
        results.push_back(get_path(sequence));
    }

    // TODO: consider adding special curves like pull over
}

/**
 * @brief Generate the space-curve following the lanelet sequence
 *
 * @param lanelets
 * @return CostedPath
 */
CostedPath PathPlanner::get_path(LaneletSequence lanelets)
{
    CostedPath path;
    for (lanelet::Id lanelet_id : lanelets)
    {
        // Todo: Do I need to check if it got lanelet successfully?
        lanelet::Lanelet lanelet = map->laneletLayer.get(lanelet_id);

        // Simple concatenate the lanelet centerline to the path.
        // This will need to be fixed for lane changes
        auto path_segment = lanelet.centerline2d();
        for (auto seg_point : path_segment)
        {
            // Todo: conversion safety from one coordinate system to another?
            geometry_msgs::msg::Point path_point;
            path_point.set__x(seg_point.x());
            path_point.set__y(seg_point.y());
            path.points.push_back(path_point);
        }
    }

    // Set route cost of path to route cost of last lanelet
    path.routing_cost = get_routing_cost(*(lanelets.end() - 1));

    return path;
}

////////////////////////////////////// Lanelet-level generation /////////////////////////////////////

void PathPlanner::get_lanelet_sequences(double distance, const geometry_msgs::msg::Point starting_point,
                                        lanelet::ConstLanelet starting_lanelet,
                                        std::vector<LaneletSequence> &results)
{

    // Recalculate remaining distance based on position in current lanelet
    //lanelet::BasicPoint2d position = ros_point_to_lanelet_point(starting_point);
    ros_point_to_lanelet_point(starting_point);

    // determine remaining length of lanelet 
    // _get_lanelet_sequences will subtract the length of starting lanelet, but
    // we have already accounted for it so add it back

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
    // add current lanelet to the sequence
    sequence.push_back(current.id());
    remaining_distance -= lanelet::geometry::length2d(current);

    if (remaining_distance <= 0)
    {
        // Don't need to  and this sequence is complete
        results.push_back(sequence);
        return;
    }

    // still need to cover some distance, look for more lanelets

    // Must assure that we don't return to the previous lane
    // Use current id as a placeholder until we're sure sequence has a previous
    lanelet::Id previous_id = current_lanelet.id();
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
            // TODO: don't remove full distance for adjacent paths
            _get_lanelet_sequences(remaining_distance, sequence, next, results);
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

    const lanelet::BasicPoint2d lanelet_position = ros_point_to_lanelet_point(pose.position);
   
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
lanelet::BasicPoint2d PathPlanner::ros_point_to_lanelet_point(geometry_msgs::msg::Point point){
    return lanelet::BasicPoint2d(point.x, point.y);
}

/**
 * @brief Checks if the two lanelets are immediately related via a left or right relation
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

    return (relation.get() == lanelet::routing::RelationType::AdjacentLeft || relation.get() == lanelet::routing::RelationType::AdjacentRight);
}