'''
Package:   path_planners
Filename:  dijkstra_path_planner.py
Author:    Bennett

Dijkstra Path Planner
'''

from matplotlib import pyplot as plt
import numpy as np
import networkx as nx
from scipy.interpolate import splrep, BSpline
import scipy.ndimage
from PIL import Image
from matplotlib.patches import Rectangle


class DijkstraPathPlanner:
    def __init__(self):
        pass

    # any cells below the threshold get expanded by padding number of cells
    def pad_obstacles(self, cmap, threshold, padding):
        # In grayscale images, black (0) is free space, white (255/100) is obstacle
        # So we need to find pixels ABOVE the threshold, not below
        obstacles = 0*cmap
        # finds the location of obstacles as binary values
        obstacles[np.where(cmap >= threshold)] = 1 
        # expands the obstacles using a given structure, repeating this padding number of times
        struct2 = scipy.ndimage.generate_binary_structure(2, 2)  # 3x3 grid all true
        obstacles = scipy.ndimage.binary_dilation(obstacles, structure=struct2, iterations=padding).astype(cmap.dtype)
        
        # Create padded costmap - set obstacle areas to 100 (white)
        padded_costmap = cmap.copy()
        padded_costmap[obstacles > 0] = 100
        
        return padded_costmap

    def shortest_path(self, costmap, start, end, obstacle_threshold=25):
        # Check if start or end is in obstacle (high value in grayscale)
        if costmap[start] >= obstacle_threshold or costmap[end] >= obstacle_threshold:
            print('Start or goal point are infeasible because they are in a high cost region (obstacle).')
            return None

        G = self.costmap_adjacency(costmap, obstacle_threshold)
        try:
            path = nx.dijkstra_path(G, start, end, 'weight')
            return path
        except:
            return None

    def costmap_adjacency(self, costmap, obstacle_threshold):
        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0, 1), (1,-1), (1,0), (1,1)]

        # create graph and add nodes (nodes are costmap grid cells)
        G = nx.DiGraph()
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                # Only add nodes for non-obstacle cells (lower than threshold in grayscale)
                if costmap[i,j] < obstacle_threshold:
                    G.add_node((i,j))
        
        # add edges from adjacent costmap cells
        for i,j in G.nodes:
            for di, dj in neighbors:
                ni, nj = i+di, j+dj
                # Check if neighbor is within bounds
                if 0 <= ni < costmap.shape[0] and 0 <= nj < costmap.shape[1]:
                    if (ni, nj) in G:
                        # Use lower values (darker) as lower costs for path planning
                        # This makes the algorithm prefer darker areas
                        weight = costmap[ni, nj]  # Use costmap value directly as weight
                        G.add_edge((i,j), (ni,nj), weight=weight)
        
        return G
        
    def rolling_smoothing(self, path, look_ahead=2, depth=3):
        """
        Smooths a path by moving each point toward a future point.
        
        Parameters:
        - path: List of (row, col) tuples representing the path
        - look_ahead: How many points ahead to look
        - depth: Number of smoothing iterations
        
        Returns:
        - Smoothed path as list of (row, col) tuples
        """
        if len(path) < look_ahead:
            return path

        t = 1.0/look_ahead
        for d in range(depth):
            new_path = [path[0]]
            for i in range(1, len(path)-look_ahead):
                x0 = path[i-1][0]
                y0 = path[i-1][1]
                
                dx = path[i-1+look_ahead][0] - x0
                dy = path[i-1+look_ahead][1] - y0
                new_path.append((x0 + t*dx, y0 + t*dy))
            new_path.extend(path[-look_ahead:])
            path = new_path
        
        return path
