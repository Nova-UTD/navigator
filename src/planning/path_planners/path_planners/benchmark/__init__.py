"""
Offline benchmark for the planners behind path_planner_node.

Runs every planner on the same costmaps through the node's own pre-planning
pipeline (planning_core.build_problem), then scores each path against an
independent exact solver. No ROS needed:

    python3 -m path_planners.benchmark --out /tmp/planner_bench

See README.md in this directory for scenarios, metrics and recorded costmaps.
"""
