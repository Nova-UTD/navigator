"""
Package:   path_planners
Filename:  dp_path_planner.py
Author:    Bennett Daniel

Dynamic Programming Path Planner
"""


import os
import io
import cv2
import time
import math
import csv
import numpy as np
import scipy.ndimage
from PIL import Image
from matplotlib import pyplot as plt
from scipy.ndimage import distance_transform_edt


class DPPathPlanner:
    def __init__(self, grid_resolution=0.4, origin_x=20.0, origin_y=30.0,
                 obstacle_threshold=25, max_iterations=1000,
                 convergence_threshold=0.0001, discount_factor=0.91,
                 smoothing_iterations=5, smoothing_weight=0.8):
        # Parameters
        self.grid_res = grid_resolution  # Grid resolution in meters/cell
        self.origin_x = origin_x  # Origin offset in X
        self.origin_y = origin_y  # Origin offset in Y
        self.obstacle_threshold = obstacle_threshold  # Values above this are considered obstacles
        self.max_iterations = max_iterations  # Maximum iterations for value iteration (changed from 500)
        self.convergence_threshold = convergence_threshold  # Threshold for convergence
        self.discount_factor = discount_factor  # Discount factor for future rewards/costs (changed from 0.95)
        # self.grid_size = 151  # This parameter is potentially misleading if grid dimensions
                                # are derived from costmap_data.shape. Commenting out.

        # Add path smoothing parameters
        self.smoothing_iterations = smoothing_iterations  # Number of smoothing passes
        self.smoothing_weight = smoothing_weight  # Weight for path smoothing (higher = smoother)

        # Movement directions (8-connected grid)
        self.actions = [
            (0, 1),  # right
            (1, 0),  # up
            (0, -1),  # left
            (-1, 0),  # down
            (1, 1),  # up-right
            (1, -1),  # up-left
            (-1, 1),  # down-right
            (-1, -1),  # down-left
        ]

        # Movement probabilities
        self.p_same_dir = 0.8  # Probability of success when continuing same direction
        self.p_change_dir = 0.6  # Probability of success when changing direction

        # Data structures for value iteration
        self.value_function = None  # Grid of state values
        self.policy = None  # Grid of optimal actions
        self.costmap_data = None  # The current costmap data

    def run_value_iteration(self, start_i, start_j, goal_i, goal_j):
        """Run an improved value iteration algorithm for dynamic programming."""
        height, width = self.costmap_data.shape

        # Initialize value function and policy
        self.value_function = np.zeros((height, width))
        self.policy = np.zeros((height, width), dtype=int)

        # Create obstacle mask - MODIFIED to match convention where higher values are obstacles
        obstacle_mask = self.costmap_data >= self.obstacle_threshold

        # Set penalties for obstacles
        self.value_function[obstacle_mask] = -1000.0

        # Set goal reward
        self.value_function[goal_i, goal_j] = 100.0

        # Create distance-based heuristic (similar to the paper's approach)
        # This creates a potential field that guides the search toward the goal
        i_indices, j_indices = np.indices((height, width))
        goal_distance = np.sqrt((i_indices - goal_i)**2 + (j_indices - goal_j)**2)
        max_distance = np.max(goal_distance)
        # Normalize distances to [0,1] range and invert
        normalized_distance = 1.0 - (goal_distance / max_distance)
        # Create heuristic with higher values closer to goal
        heuristic = 20.0 * normalized_distance
        
        # Create reward map with costmap values and heuristic
        normalized_cost = (self.costmap_data) / 100.0  # Convert to 0-1 range
        cost_penalty = -5.0 * normalized_cost  # Scale to make it significant
        
        # Create base reward map
        reward_map = -1.0 * np.ones_like(self.costmap_data)  # Base cost for movement
        reward_map += cost_penalty  # Add costmap-based penalties
        reward_map += heuristic    # Add goal-directed heuristic
        reward_map[obstacle_mask] = -1000.0  # Obstacle penalty
        reward_map[goal_i, goal_j] = 100.0  # Goal reward

        # Create directional bias toward goal (as in the paper)
        goal_direction_i = goal_i - i_indices
        goal_direction_j = goal_j - j_indices
        direction_norm = np.sqrt(goal_direction_i**2 + goal_direction_j**2) + 1e-10
        goal_direction_i = goal_direction_i / direction_norm
        goal_direction_j = goal_direction_j / direction_norm

        # Identify road area for focused computation
        road_mask = self.costmap_data > 50
        buffer_size = 5  # cells
        road_mask = scipy.ndimage.binary_dilation(road_mask, iterations=buffer_size)
        road_mask[start_i, start_j] = True
        road_mask[goal_i, goal_j] = True
        compute_indices = np.argwhere(road_mask)

        # Value iteration with adaptive learning rate (inspired by the paper)
        learning_rate = 1.0  # Initial learning rate
        min_learning_rate = 0.6  # Minimum learning rate
        
        for iteration in range(self.max_iterations):
            delta = 0
            value_old = np.copy(self.value_function)

            # Adaptive learning rate decay (similar to the paper's approach)
            if iteration > 0 and iteration % 20 == 0:
                learning_rate = max(min_learning_rate, learning_rate * 0.95)

            # Only iterate over road cells for better performance
            for idx in compute_indices:
                i, j = idx
                if obstacle_mask[i, j] or (i == goal_i and j == goal_j):
                    continue

                best_value = float("-inf")
                best_action = 0

                # Try all actions
                for action_idx, (di, dj) in enumerate(self.actions):
                    next_i, next_j = i + di, j + dj

                    if (
                        0 <= next_i < height
                        and 0 <= next_j < width
                        and not obstacle_mask[next_i, next_j]
                    ):
                        # Calculate alignment with goal direction (from the paper)
                        action_alignment = di * goal_direction_i[i, j] + dj * goal_direction_j[i, j]
                        alignment_bonus = 1.0 * action_alignment  # Stronger alignment bonus

                        # Add a smaller penalty for diagonal moves
                        move_penalty = 0.0
                        if di != 0 and dj != 0:  # Diagonal move
                            move_penalty = -0.1

                        # Add transition cost based on costmap
                        next_cell_cost = -0.2 * normalized_cost[next_i, next_j]
                        
                        # Calculate the total value for this action
                        action_value = (
                            reward_map[i, j]
                            + move_penalty
                            + alignment_bonus
                            + next_cell_cost
                            + self.discount_factor * value_old[next_i, next_j]
                        )

                        if action_value > best_value:
                            best_value = action_value
                            best_action = action_idx

                if best_value > float("-inf"):
                    # Apply learning rate to update (similar to the paper's approach)
                    new_value = (1 - learning_rate) * self.value_function[i, j] + learning_rate * best_value
                    self.value_function[i, j] = new_value
                    self.policy[i, j] = best_action
                    delta = max(delta, abs(new_value - value_old[i, j]))

            if delta < self.convergence_threshold:
                # Convergence achieved
                return True

        # Value iteration did not converge, but we'll still try to return a path
        return True  # Still return a path even if not fully converged

        # Even if we didn't fully converge, check if there's a valid path
        test_path = self.extract_path((start_i, start_j), (goal_i, goal_j))
        if test_path is not None and len(test_path) > 0:
            return True

        return False

    def extract_path(self, start, goal):
        """Extract the path from the policy grid using improved techniques."""
        if self.policy is None:
            # Policy not computed yet
            return None

        path = [start]
        current = start
        visited = set([start])  # Track visited states to prevent loops

        # Maximum steps to prevent infinite loops
        max_steps = min(2000, self.costmap_data.shape[0] * self.costmap_data.shape[1])
        step_count = 0
        
        # Create a distance map to the goal for backup heuristic
        height, width = self.costmap_data.shape
        goal_i, goal_j = goal
        i_indices, j_indices = np.indices((height, width))
        goal_distance = np.sqrt((i_indices - goal_i)**2 + (j_indices - goal_j)**2)
        
        # Track progress toward the goal
        last_distance_to_goal = np.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)
        no_progress_count = 0
        
        # Create a priority queue for alternative paths (inspired by the paper)
        alternative_paths = []
        backtrack_points = []
        
        while current != goal and step_count < max_steps:
            i, j = current
            
            # Special case: If we're very close to the goal, go directly to it
            current_to_goal_dist = np.sqrt((i - goal[0])**2 + (j - goal[1])**2)
            if current_to_goal_dist <= 2.0:  # If within 2 cells of goal
                # Check if direct path to goal is obstacle-free
                    if not self.line_crosses_obstacle(current, goal, self.costmap_data >= self.obstacle_threshold):
                        # Go directly to goal
                        path.append(goal)
                        return path
            
            action_idx = self.policy[i, j]  # Get action from policy
            
            # If we're not making progress, try alternative paths
            if no_progress_count > 3:
                # Find all valid actions sorted by value
                valid_actions = []
                for idx, (di, dj) in enumerate(self.actions):
                    ni, nj = i + di, j + dj
                    if (0 <= ni < height and 0 <= nj < width and 
                        self.costmap_data[ni, nj] < self.obstacle_threshold and
                        (ni, nj) not in visited):
                        # Calculate distance to goal and value
                        new_dist = np.sqrt((ni - goal[0])**2 + (nj - goal[1])**2)
                        cell_value = self.value_function[ni, nj]
                        valid_actions.append((idx, new_dist, cell_value, (ni, nj)))
                
                if valid_actions:
                    # Sort by distance to goal (lower is better)
                    valid_actions.sort(key=lambda x: x[1])
                    
                    # Save alternative paths for backtracking
                    for alt in valid_actions[1:]:  # Skip the best one
                        alternative_paths.append((current, alt[0], alt[3]))
                    
                    # Choose best action
                    action_idx = valid_actions[0][0]
                    backtrack_points.append(len(path) - 1)  # Mark this as a decision point
                    no_progress_count = 0
            
            action = self.actions[action_idx]
            next_i = i + action[0]
            next_j = j + action[1]
            next_state = (next_i, next_j)

            # Check bounds and obstacles
            if not (0 <= next_i < height and 0 <= next_j < width):
                # Path went out of bounds
                break
                
            if self.costmap_data[next_i, next_j] >= self.obstacle_threshold:
                # Path hit an obstacle
                break

            # Check for loops
            if next_state in visited:
                # Try to find an alternative path
                if alternative_paths:
                    # Get the most recent alternative
                    alt_start, alt_action_idx, alt_next = alternative_paths.pop()
                    
                    # Backtrack to the decision point
                    if backtrack_points:
                        backtrack_idx = backtrack_points.pop()
                        path = path[:backtrack_idx + 1]
                        visited = set(path)
                        current = alt_start
                        
                        # Take the alternative action
                        alt_action = self.actions[alt_action_idx]
                        next_i = current[0] + alt_action[0]
                        next_j = current[1] + alt_action[1]
                        next_state = alt_next
                        
                        # Loop detected, trying alternative path
                        no_progress_count = 0
                    else:
                        # If we're close to the goal, try direct path
                        if np.sqrt((i - goal[0])**2 + (j - goal[1])**2) < 5:
                            # Try to go directly to the goal
                            if not self.line_crosses_obstacle(current, goal, self.costmap_data >= self.obstacle_threshold):
                                path.append(goal)
                                return path
                        
                        # Path has a loop and no valid backtrack point
                        break
                else:
                    # Last resort: If we're close to goal, try direct path
                    if np.sqrt((i - goal[0])**2 + (j - goal[1])**2) < 5:
                        # Try to go directly to the goal
                        if not self.line_crosses_obstacle(current, goal, self.costmap_data >= self.obstacle_threshold):
                            path.append(goal)
                            return path
                        
                        # Path has a loop and no alternatives available
                        break

            # Move to next state
            current = next_state
            path.append(current)
            visited.add(current)

            # Check if we're making progress toward the goal
            current_distance = np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)
            
            # If we're very close to the goal but not exactly at it, and we're not making progress
            # This helps with the "one step before goal" loop issue
            if current_distance < 3 and current_distance >= last_distance_to_goal:
                # Try to go directly to the goal
                if not self.line_crosses_obstacle(current, goal, self.costmap_data >= self.obstacle_threshold):
                    path.append(goal)
                    return path
            
            if current_distance >= last_distance_to_goal:
                no_progress_count += 1
            else:
                no_progress_count = 0
            last_distance_to_goal = current_distance

            step_count += 1

        if step_count >= max_steps:
            # Path extraction reached maximum steps
            return None

        if current != goal:
            # Could not reach goal during path extraction
            return None

        return path

    def line_crosses_obstacle(self, p1, p2, obstacle_mask):
        """Check if a line segment crosses any obstacles."""
        # Use Bresenham's line algorithm to check all cells along the line
        x1, y1 = int(round(p1[0])), int(round(p1[1]))
        x2, y2 = int(round(p2[0])), int(round(p2[1]))

        # Get all points on the line
        points = self.bresenham_line(x1, y1, x2, y2)

        # Check if any point is an obstacle
        for x, y in points:
            if 0 <= x < obstacle_mask.shape[0] and 0 <= y < obstacle_mask.shape[1]:
                if obstacle_mask[x, y]:
                    return True

        return False

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for getting all points on a line."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    # Keep the existing smooth_path method but rename it to avoid conflicts
    def adaptive_smooth_path(self, path):
        """Apply a path smoothing algorithm that respects obstacles."""
        if path is None or len(path) < 3:
            return path

        # Convert path to numpy array for easier manipulation
        path_array = np.array(path, dtype=float)

        # Create obstacle mask
        obstacle_mask = self.costmap_data >= self.obstacle_threshold

        # Adaptive smoothing - use less aggressive smoothing at turns
        for _ in range(self.smoothing_iterations):
            # Skip first and last points (keep them fixed)
            for i in range(1, len(path_array) - 1):
                # Calculate direction changes to detect turns
                if i > 1 and i < len(path_array) - 2:
                    prev_dir = path_array[i] - path_array[i - 1]
                    next_dir = path_array[i + 1] - path_array[i]

                    # Normalize directions
                    prev_dir_norm = np.linalg.norm(prev_dir) + 1e-10
                    next_dir_norm = np.linalg.norm(next_dir) + 1e-10
                    prev_dir = prev_dir / prev_dir_norm
                    next_dir = next_dir / next_dir_norm

                    # Calculate dot product to measure direction change
                    dir_change = np.dot(prev_dir, next_dir)

                    # Adjust smoothing weight based on direction change
                    # Less smoothing at turns (when dir_change is low)
                    adaptive_weight = self.smoothing_weight * (0.5 + 0.5 * dir_change)
                else:
                    adaptive_weight = self.smoothing_weight

                # Calculate smoothed position with adaptive weight
                smoothed_pos = (1 - adaptive_weight) * path_array[
                    i
                ] + adaptive_weight * 0.5 * (path_array[i - 1] + path_array[i + 1])

                # Round to get grid indices
                si, sj = int(round(smoothed_pos[0])), int(round(smoothed_pos[1]))

                # Check if the smoothed position is valid (within bounds and not an obstacle)
                height, width = self.costmap_data.shape
                if 0 <= si < height and 0 <= sj < width and not obstacle_mask[si, sj]:
                    # Check if the line segment to the smoothed position crosses any obstacles
                    if not self.line_crosses_obstacle(
                        path_array[i - 1], smoothed_pos, obstacle_mask
                    ) and not self.line_crosses_obstacle(
                        smoothed_pos, path_array[i + 1], obstacle_mask
                    ):
                        # Update the path point
                        path_array[i] = smoothed_pos

        # Convert back to list of tuples
        smoothed_path = [(int(point[0]), int(point[1])) for point in path_array]

        # Check if smoothing made the path too short (can happen with aggressive smoothing)
        if len(smoothed_path) < 3:
            # Smoothing made path too short, using original path
            return path

        return smoothed_path