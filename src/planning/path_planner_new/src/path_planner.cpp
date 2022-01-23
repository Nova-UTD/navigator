
/* Todo list for this file (roughly sorted most important first)
 *  - Implement safety cost (obstacles etc)
 *  - better guess logic for current lanelet (use predicted path)
 *  - Implementation for short/choppy lanelets
 *  - Better horizon (see .hpp)
 *  - Special non-cruising paths like pull over
 *  - Account for horizontal lane change distance in lanelet sequence generation
 *  - Move visualization to a different nodes
 * 
 *  - Documentation
 *  - Copied a lot of code from Lanelet2GlobalPlanner and I think there's a better
 *    way to reuse it. Applies to PathPlannerNode as well.
 *  - Copyright info
 *
 */
#include <path_planner_new/path_planner.hpp>

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

void PathPlanner::get_paths(
    const geometry_msgs::msg::Pose &current_pose,
    const std::vector<RouteCost> &route_costs,
    std::vector<CostedPath> &results)
{
    // Update current state using setters
    log("Marker get1");
    set_routing_costs(route_costs);
    log("Marker get2 hello");
    set_pose(current_pose);
    log("Marker get3");

    // Grab starting location and use it to generate all lanelet-level paths
    auto current_lanelet = get_current_lanelet();
    log("Marker get4");

    double horizon = get_horizon();
    log("Marker get5");
    auto pos = current_pose.position;
    log("Marker get5b");

    std::vector<LaneletSequence> lanelet_sequences;
    get_lanelet_sequences(horizon, pos, current_lanelet, lanelet_sequences);
    log("Marker get6");

    for (auto sequence : lanelet_sequences)
    {
        results.push_back(get_path(sequence));
    }
    log("Marker get7");
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

void PathPlanner::set_map(lanelet::LaneletMapPtr map)
{
    this->map = map;

    // generate routing graph
    routing_graph.release();
    lanelet::traffic_rules::TrafficRulesPtr trafficRules =
        lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, // TODO: change locations
            lanelet::Participants::Vehicle);
    log("Routing graph created");
    routing_graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);
}

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
    log("Marker pose1");
    const lanelet::Point2d lanelet_position(lanelet::utils::getId(), pose.position.x, pose.position.y, 0.0);
    if (lanelet::geometry::inside(current_lanelet, lanelet_position))
    {
            log("Marker pose exit");

        // retain current lanelet
        return;
    }
    log("Marker pose2");
    // Todo: Made up 5 on the spot. Find a better number like max conflicts in map
    lanelet::Lanelets lanelets = map->laneletLayer.nearest(lanelet_position, 5);
    log("Marker pose3");

    // Linear search for highest priority and best pose match
    // Order by higher priority and then by higher pose match
    lanelet::Lanelet best_match_lanelet;
    LaneletPriority priority = LaneletPriority::NOT_SET;
    double pose_match = std::numeric_limits<double>::min();
    log("Marker pose4");

    for (auto lanelet : lanelets)
    {
        log(std::to_string(lanelet.id()));
        LaneletPriority curr_priority = LaneletPriority::NOT_SET;
        double curr_pose_match = -std::numeric_limits<double>::max();

        if (lanelet::geometry::inside(lanelet, lanelet_position))
        {
            log("Containing");
            // TODO: check predicted
            curr_priority = LaneletPriority::CONTAINING;
        }
        else
        {
            log("Nearby");
            curr_priority = LaneletPriority::NEARBY;
        }

        // Calculate pose metric.
        // TODO: incorperate orientation. Distance only for now.
        curr_pose_match = -lanelet::geometry::distanceToCenterline2d(lanelet, lanelet_position);

        log("Comparing current prio " + std::to_string(curr_priority) + " and match " + std::to_string(curr_pose_match) + "to " + std::to_string(priority) + " , " + std::to_string(pose_match));
        if ((curr_priority > priority) || ((curr_priority == priority) && (curr_pose_match > pose_match)))
        {
            log("Setting");
            // have new best lanelet
            best_match_lanelet = lanelet;
            priority = curr_priority;
            pose_match = curr_pose_match;
        }
    }
    log("New current lanelet with id");
    log(std::to_string(best_match_lanelet.id()));
    // Update current with new best guess
    current_lanelet = best_match_lanelet;
}

void PathPlanner::get_lanelet_sequences(double distance, const geometry_msgs::msg::Point starting_point,
                                        lanelet::ConstLanelet starting_lanelet,
                                        std::vector<LaneletSequence> &results)
{
    // Todo: calculate remaining distance based on position in current lanelet
    log("Marker seq1");
    _get_lanelet_sequences(distance, LaneletSequence(), starting_lanelet, results);
    log("Number of sequences:");
    log(std::to_string(results.size()));
    // useless code to get it to compile
    if (starting_point.z > 0)
    {
        log("z doesn't matter");
    }
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
    log("Marker _seq1");
    // add current lanelet to the sequence
    sequence.push_back(current.id());
    remaining_distance -= lanelet::geometry::length2d(current);

    log("Marker _seq2");

    if (remaining_distance <= 0)
    {
        // out of distance and this sequence is complete
        log("Marker _seq3 cond1");

        results.push_back(sequence);
    }
    else
    {
        log("Marker _seq3 cond1");

        // still need to cover some distance, look for more lanelets

        // Must assure that we don't return to the previous lane
        // TODO: wanted to make sure didn't exceed indices. Make cleaner
        lanelet::Id previous_id = current_lanelet.id();
        if (sequence.size() > 1)
        {
            previous_id = *(sequence.end() - 2);
        }

        log("Marker _seq4");

        // Successors and adjacents both included in following
        auto relations = routing_graph->following(current, true);
        log("Marker _seq5");
        log(std::to_string(relations.size()));

        for (lanelet::ConstLanelet next : relations)
        {
            log("Successor");
            if (next.id() != previous_id)
            {
                log("branching sequence");
                _get_lanelet_sequences(remaining_distance, sequence, next, results);
            }
        }
        log("Marker _seq6");
    }
    log("Marker _seq7");
}

lanelet::Lanelet PathPlanner::get_current_lanelet()
{
    return current_lanelet;
}

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