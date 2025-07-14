"""
Package:   path_planners
Filename:  arastar_path_planner.py
Author:    Bennett Daniel

ARA* Path Planner
"""

import math
import numpy as np
import networkx as nx
from PIL import Image
import io
import cv2
from matplotlib import pyplot as plt
import scipy.ndimage


class ARAStarPlanner:
    def __init__(self, e=2.5, e_delta=0.5, e_min=1.0):
        # Parameters
        self.e = e  # Weight factor for suboptimality
        self.e_delta = e_delta
        self.e_min = e_min
        self.heuristic_type = "euclidean"
        self.obstacle_threshold = 25
        self.s_start = None
        self.s_goal = None
        self.cmap = None
        self.u_set = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),  # 4-way movement
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]  # Diagonal movement
        self.g = {}  # Cost-to-come
        self.OPEN = {}  # Priority queue
        self.CLOSED = set()  # Closed set
        self.INCONS = {}  # Inconsistent set
        self.PARENT = {}  # Parent tracking
        self.path = []  # Planned path
        self.visited = []  # Order of visited nodes
        self.G = nx.DiGraph()

    def create_graph_from_costmap(self):
        """Create a graph from the costmap using NetworkX."""
        self.G.clear()
        for i in range(self.cmap.shape[0]):
            for j in range(self.cmap.shape[1]):
                # Only add nodes for non-obstacle cells (lower than threshold in grayscale)
                if self.cmap[i,j] < self.obstacle_threshold:
                    self.G.add_node((i,j))
        
        # add edges from adjacent costmap cells
        for i,j in self.G.nodes:
            for di, dj in self.u_set:
                ni, nj = i+di, j+dj
                # Check if neighbor is within bounds
                if 0 <= ni < self.cmap.shape[0] and 0 <= nj < self.cmap.shape[1]:
                    if (ni, nj) in self.G:
                        # Use lower values (darker) as lower costs for path planning
                        weight = self.cmap[ni, nj]  # Use costmap value directly as weight
                        self.G.add_edge((i,j), (ni,nj), weight=weight)

    def arastar(self):
        """Main ARA* search loop using NetworkX."""
        # Reset data structures for a fresh search
        self.g = {}
        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = float("inf")
        self.OPEN = {self.s_start: self.f_value(self.s_start)}
        self.CLOSED = set()
        self.INCONS = {}
        self.PARENT = {self.s_start: self.s_start}

        # Check if start and goal are in the graph
        if self.s_start not in self.G or self.s_goal not in self.G:
            # print(f"Start or goal node not in graph: Start: {self.s_start in self.G}, Goal: {self.s_goal in self.G}")
            return None

        # Check graph connectivity
        if not nx.has_path(self.G, self.s_start, self.s_goal):
            # print(f"No path exists between start and goal in the graph")
            return None

        self.ImprovePath()
        path = self.extract_path()
        if path is None:
            # print("Path extraction failed in initial search")
            return None

        current_path = path

        while self.update_e() > self.e_min:
            self.e -= self.e_delta  # Decrease weight for better solution
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}
            self.INCONS = {}
            self.CLOSED = set()
            self.ImprovePath()
            new_path = self.extract_path()
            if new_path is not None:
                current_path = new_path

        return current_path

    def ImprovePath(self):
        """Refines the path based on updated information."""
        visited_each = []

        while self.OPEN:
            s, f_small = self.calc_smallest_f()
            if self.f_value(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbors(s):
                # Calculate cost only for valid neighbors in the graph
                if s in self.G and s_n in self.G and s_n in self.G[s]:
                    # Use the edge weight from the graph
                    edge_cost = self.G[s][s_n]["weight"]
                    new_cost = self.g[s] + edge_cost

                    if s_n not in self.g or new_cost < self.g[s_n]:
                        self.g[s_n] = new_cost
                        self.PARENT[s_n] = s
                        visited_each.append(s_n)

                        if s_n not in self.CLOSED:
                            self.OPEN[s_n] = self.f_value(s_n)
                        else:
                            self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """Returns the node with the smallest f-value in OPEN."""
        s_small = min(self.OPEN, key=self.OPEN.get)
        return s_small, self.OPEN[s_small]

    def get_neighbors(self, s):
        """Returns valid neighbors of the given state that exist in the graph."""
        if s not in self.G:
            return []

        return list(self.G.neighbors(s))

    def update_e(self):
        """Updates the weight factor e for improved paths."""
        if not self.OPEN and not self.INCONS:
            return self.e

        v = float("inf")
        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)

    def f_value(self, s):
        """Calculates f = g + e * h (cost-to-come + heuristic)."""
        return self.g[s] + self.e * self.h(s)

    def extract_path(self):
        """Backtracks from the goal to reconstruct the path."""
        if self.s_goal not in self.PARENT and self.s_goal not in self.g:
            # print(f"Goal node {self.s_goal} not reached")
            return None

        path = [self.s_goal]
        s = self.s_goal

        while s != self.s_start:
            if s not in self.PARENT:
                # print(f"Path extraction failed, no parent for node: {s}")
                return None
            s = self.PARENT[s]
            path.append(s)

        path.reverse()  # Ensure it's ordered from start to goal
        return path

    def h(self, s):
        """Heuristic function for path estimation."""
        goal = self.s_goal

        if self.heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])
