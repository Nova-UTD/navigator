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
