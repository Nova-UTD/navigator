"""
Package:   path_planners
Filename:  trrtstar_path_planner.py
Author:    Bennett Daniel

TRRT* Path Planner

This implementation of the TRRT* (Transition-based Rapidly-exploring Random Tree Star) path planning algorithm is based on the official implementation in the OMPL library, specifically from the following repositories:
- https://github.com/sirojc/ompl/blob/dev/sirojc/optimal-trrt/src/ompl/geometric/planners/rrt/src/TRRTstar.cpp
- https://github.com/sirojc/ompl/blob/dev/sirojc/optimal-trrt/src/ompl/geometric/planners/rrt/TRRTstar.h

This official implementation is detailed in the following paper:
Didier Devaurs, Thierry Siméon, Juan Cortés. Efficient sampling-based approaches to optimal path planning in complex cost spaces. Algorithmic Foundations of Robotics XI (selected contributions from WAFR ‘14), 2015: pages 143-159; Springer, DOI: 10.1007/978-3-319-16595-0_9
Paper Link: https://link.springer.com/content/pdf/10.1007/978-3-319-16595-0_9.pdf
"""

from matplotlib import pyplot as plt
import numpy as np

class TRRTStarPathPlanner:
    def __init__(self):
        pass

    def trrtstar_path(self, costmap, start, end, obstacle_threshold=25, max_iterations=30000, step_size=2, goal_sample_rate=0.1, search_radius=35, rewire_factor=2.0, temp_init=2.0, temp_factor=1.0):
        """
        TRRT* (Transition-based Rapidly-exploring Random Tree Star) path planning algorith.
        
        Parameters:
        - costmap: 2D numpy array representing the environment (higher values = more traversable)
        - start: Tuple (row, col) of start position
        - end: Tuple (row, col) of goal position
        - obstacle_threshold: Cells with values below this are considered obstacles
        - max_iterations: Maximum number of iterations for the algorithm
        - step_size: Maximum distance between nodes
        - goal_sample_rate: Probability of sampling the goal position
        - search_radius: Radius for finding nearby nodes for rewiring
        - rewire_factor: Factor to scale the search radius
        - temp_init: Initial temperature for transition tests
        - temp_factor: Factor to scale temperature
        
        Returns:
        - List of (row, col) tuples representing the path from start to end
        """
        import math
        import random
        from collections import defaultdict
        
        # Node class for the tree
        class Node:
            def __init__(self, row, col):
                self.row = row
                self.col = col
                self.cost = 0.0  # Cost from start to this node
                self.parent = None
            
            def __eq__(self, other):
                if isinstance(other, Node):
                    return self.row == other.row and self.col == other.col
                return False
                
            def position(self):
                return (self.row, self.col)
        
        # Initialize tree with start node
        start_node = Node(start[0], start[1])
        end_node = Node(end[0], end[1])
        nodes = [start_node]
        
        # Temperature for transition tests - increased initial temperature
        temperature = temp_init * 35  # Higher initial temperature for better exploration
        
        # Dictionary to track nodes by position for quick lookup
        node_dict = {start: start_node}
        
        # Track failed transitions for adaptive temperature
        failed_transitions = 0
        max_failed_transitions = 100  # Threshold for temperature adjustment
        
        # Helper functions
        def distance(node1, node2):
            return math.sqrt((node1.row - node2.row)**2 + (node1.col - node2.col)**2)
        
        def cost_at(row, col):
            # Lower values in costmap = lower cost for planning
            if 0 <= row < costmap.shape[0] and 0 <= col < costmap.shape[1]:
                # Normalize cost to be between 0 and 1 for cells below threshold
                raw_cost = costmap[int(row), int(col)]
                if raw_cost <= obstacle_threshold:
                    return raw_cost / obstacle_threshold  # Normalize
                return float('inf')
            return float('inf')
        
        def is_collision_free(node1, node2):
            # Check if the line between node1 and node2 is collision-free
            n_steps = max(1, int(distance(node1, node2)))
            for i in range(n_steps + 1):
                t = i / n_steps
                row = int(node1.row + t * (node2.row - node1.row))
                col = int(node1.col + t * (node2.col - node1.col))
                
                # Check bounds and obstacle threshold
                if not (0 <= row < costmap.shape[0] and 0 <= col < costmap.shape[1]):
                    return False
                if costmap[row, col] > obstacle_threshold:  # This is correct - checking if value is >= threshold
                    return False
            return True
        
        def transition_test(c_near, c_new, d):
            # Improved Metropolis criterion for accepting uphill transitions
            if c_new <= c_near:
                return True

            if c_new <= 0.8:  # If cost is below 80% of threshold
                return True
            
            delta_c = c_new - c_near
            # Normalize delta_c by distance to avoid bias towards short edges
            normalized_delta = delta_c / (d + 0.1)  # Avoid division by zero
            p = math.exp(-normalized_delta / temperature)
            return random.random() < p
        
        def sample_random_point():
            # Sample random point or goal with probability goal_sample_rate
            if random.random() < goal_sample_rate:
                return end
            
            # Improved sampling strategy - focus on areas near the start and goal
            if random.random() < 0.3:  # 30% chance to sample near start or goal
                if random.random() < 0.5:  # 50% chance for start vs goal
                    center = start
                else:
                    center = end
                
                # Sample within a radius around the chosen point
                radius = random.uniform(0, costmap.shape[0] / 4)
                angle = random.uniform(0, 2 * math.pi)
                row = int(center[0] + radius * math.sin(angle))
                col = int(center[1] + radius * math.cos(angle))
                
                # Ensure within bounds
                row = max(0, min(row, costmap.shape[0] - 1))
                col = max(0, min(col, costmap.shape[1] - 1))
                
                return (row, col)
            
            # Otherwise sample uniformly
            row = random.randint(0, costmap.shape[0] - 1)
            col = random.randint(0, costmap.shape[1] - 1)
            return (row, col)
        
        def nearest_node(point):
            # Find the nearest node in the tree to the given point
            min_dist = float('inf')
            nearest = None
            for node in nodes:
                d = math.sqrt((node.row - point[0])**2 + (node.col - point[1])**2)
                if d < min_dist:
                    min_dist = d
                    nearest = node
            return nearest
        
        def steer(from_node, to_point):
            # Steer from node towards point with maximum step_size
            d = math.sqrt((from_node.row - to_point[0])**2 + (from_node.col - to_point[1])**2)
            
            if d <= step_size:
                return Node(to_point[0], to_point[1])
            
            theta = math.atan2(to_point[0] - from_node.row, to_point[1] - from_node.col)
            new_row = from_node.row + step_size * math.sin(theta)
            new_col = from_node.col + step_size * math.cos(theta)
            
            return Node(int(new_row), int(new_col))
        
        def near_nodes(node, radius):
            # Find nodes within radius of the given node
            result = []
            for n in nodes:
                if distance(node, n) <= radius and n != node:
                    result.append(n)
            return result
        
        # Main TRRT* loop
        for i in range(max_iterations):
            # Sample random point
            random_point = sample_random_point()
            
            # Find nearest node
            nearest = nearest_node(random_point)
            
            # Steer towards random point
            new_node = steer(nearest, random_point)
            
            # Skip if this position already exists in the tree
            if new_node.position() in node_dict:
                continue
                
            # Check if path is collision-free
            if not is_collision_free(nearest, new_node):
                continue
            
            # Calculate costs
            c_near = cost_at(nearest.row, nearest.col)
            c_new = cost_at(new_node.row, new_node.col)
            d = distance(nearest, new_node)
            
            # Transition test (for handling cost spaces)
            if not transition_test(c_near, c_new, d):
                # Adjust temperature if transition is rejected
                failed_transitions += 1
                
                # Adaptive temperature adjustment
                if failed_transitions > max_failed_transitions:
                    temperature *= 1.1  # Increase temperature to accept more transitions
                    failed_transitions = 0
                continue
            
            # Reset failed transitions counter on success
            failed_transitions = 0
            
            # Calculate radius for near nodes (RRT* part)
            # Improved radius calculation based on the number of nodes
            search_r = min(search_radius, rewire_factor * math.pow(math.log(len(nodes) + 1) / (len(nodes) + 1), 0.5))
            
            # Find nearby nodes for potential rewiring
            near_list = near_nodes(new_node, search_r)
            
            # Connect new_node to the parent that results in lowest cost
            min_cost = nearest.cost + d * (1 + 0.1 * c_new)  # Consider cost at new node
            min_parent = nearest
            
            def edge_cost(node1, node2):
                # Compute the average cost along the edge from node1 to node2
                n_steps = max(1, int(distance(node1, node2)))
                total = 0.0
                for i in range(n_steps + 1):
                    t = i / n_steps
                    row = int(node1.row + t * (node2.row - node1.row))
                    col = int(node1.col + t * (node2.col - node1.col))
                    if not (0 <= row < costmap.shape[0] and 0 <= col < costmap.shape[1]):
                        return float('inf')
                    total += costmap[row, col]
                return total / (n_steps + 1) * distance(node1, node2)  # Weighted by distance
            
            # In the main loop, when selecting parent:
            min_cost = nearest.cost + edge_cost(nearest, new_node)
            min_parent = nearest
            for near_node in near_list:
                if is_collision_free(near_node, new_node):
                    cost = near_node.cost + edge_cost(near_node, new_node)
                    if cost < min_cost:
                        min_cost = cost
                        min_parent = near_node
            new_node.parent = min_parent
            new_node.cost = min_cost
            
            # In rewiring:
            for near_node in near_list:
                if near_node != min_parent:
                    if is_collision_free(new_node, near_node):
                        cost = new_node.cost + edge_cost(new_node, near_node)
                        if cost < near_node.cost:
                            near_node.parent = new_node
                            near_node.cost = cost
            
            # Add new node to tree
            nodes.append(new_node)
            node_dict[new_node.position()] = new_node
            
            # Rewire tree (RRT* part)
            for near_node in near_list:
                if near_node != min_parent:
                    if is_collision_free(new_node, near_node):
                        near_d = distance(new_node, near_node)
                        c_near_node = cost_at(near_node.row, near_node.col)
                        cost = new_node.cost + near_d * (1 + 0.1 * c_near_node)
                        if cost < near_node.cost:
                            near_node.parent = new_node
                            near_node.cost = cost
            
            # Check if we can connect to goal - increased connection radius
            if distance(new_node, end_node) < step_size * 2:  # Increased connection radius
                if is_collision_free(new_node, end_node):
                    end_node.parent = new_node
                    end_node.cost = new_node.cost + distance(new_node, end_node)
                    nodes.append(end_node)
                    node_dict[end_node.position()] = end_node
                    break
            
            # Periodically try to connect to goal
            if i % 100 == 0 and distance(new_node, end_node) < step_size * 5:
                if is_collision_free(new_node, end_node):
                    end_node.parent = new_node
                    end_node.cost = new_node.cost + distance(new_node, end_node)
                    nodes.append(end_node)
                    node_dict[end_node.position()] = end_node
                    break
        
        # Extract path by backtracking from goal to start
        if end_node.parent is None:
            # Try to find the closest node to goal
            closest_node = None
            min_dist = float('inf')
            for node in nodes:
                d = distance(node, end_node)
                if d < min_dist:
                    min_dist = d
                    closest_node = node
            
            # If we found a close node, create a path to it
            if closest_node is not None and min_dist < step_size * 5:
                path = []
                current = closest_node
                while current is not None:
                    path.append((current.row, current.col))
                    current = current.parent
                
                # Add goal to the path
                path.append(end)
                
                # Reverse path to get start-to-goal order
                path.reverse()
                return path
            
            # No path found
            return None
        
        path = []
        current = end_node
        while current is not None:
            path.append((current.row, current.col))
            current = current.parent
        
        # Reverse path to get start-to-goal order
        path.reverse()
        
        return path
