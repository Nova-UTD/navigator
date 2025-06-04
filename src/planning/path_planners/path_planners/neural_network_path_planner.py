"""
Package:   path_planners
Filename:  Neural_network.py
Author:    Bennett Daniel

Neural Network Path Planner
"""

import io
import os
import sys
import numpy as np
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F  
from skimage.morphology import skeletonize
import scipy.ndimage
import time
from PIL import Image

# Add the parent directory to the path so we can import the neural network module
sys.path.append('/navigator/src/planning/path_planners/path_planners')
# Import the correct model class name: UNet
from neuralnet import UNet

class NeuralPathPlanner:
    def __init__(self, model_path=None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # Instantiate the UNet model
        # n_channels=3 because UNet's forward method ex pects costmap, start, goal concatenated
        # n_classes=1 because the output is a single-channel path probability map
        self.model = UNet(n_channels=3, n_classes=1).to(self.device)

        # Load pre-trained model if available
        if model_path and os.path.exists(model_path):
            try:
                # Load the state dict
                self.model.load_state_dict(torch.load(model_path, map_location=self.device))
                self.model.eval()
                print(f"Successfully loaded UNet model from {model_path}") # Add confirmation
            except RuntimeError as e:
                # Add more informative error message for architecture mismatch
                print(f"Error loading model state_dict from {model_path}: {e}")
                print("This might be due to an architecture mismatch. Ensure the saved model corresponds to the UNet architecture.")
                # Optionally, prevent the app from continuing or use a default state
                # For now, we'll let it proceed but the model won't be loaded correctly.
                # Consider adding st.error() in the main app part if loading fails.
            except Exception as e:
                print(f"An unexpected error occurred loading the model: {e}")
        else:
            print(f"Model file not found at {model_path}. Planner initialized with untrained UNet.")

    def pad_obstacles(self, cmap, threshold, padding):
        # In grayscale images, black (0) is obstacle, white (255/100) is free space
        obstacles = 0*cmap
        # finds the location of obstacles as binary values
        obstacles[np.where(cmap <= threshold)] = 1 
        # expands the obstacles using a given structure, repeating this padding number of times
        struct2 = scipy.ndimage.generate_binary_structure(2, 2)  # 3x3 grid all true
        obstacles = scipy.ndimage.binary_dilation(obstacles, structure=struct2, iterations=padding).astype(cmap.dtype)
        
        # Create padded costmap - set obstacle areas to 0 (black)
        padded_costmap = cmap.copy()
        padded_costmap[obstacles > 0] = 0
        
        return padded_costmap
    
    def predict_path(self, costmap, start, end):
        """
        Use the neural network to predict a path from start to end
        
        Parameters:
        - costmap: 2D numpy array representing the environment
        - start: Tuple (row, col) of start position
        - end: Tuple (row, col) of goal position
        
        Returns:
        - List of (row, col) tuples representing the path from start to end
        """
        # Normalize costmap to [0, 1]
        costmap_norm = costmap / 100.0
        
        # Prepare input for the model
        costmap_tensor = torch.tensor(costmap_norm, dtype=torch.float32).unsqueeze(0).unsqueeze(0).to(self.device)
        start_tensor = torch.tensor([start], dtype=torch.float32).to(self.device)
        end_tensor = torch.tensor([end], dtype=torch.float32).to(self.device)
        
        # Get prediction from the model
        with torch.no_grad():
            path_prob_map = self.model(costmap_tensor, start_tensor, end_tensor).squeeze().cpu().numpy()
        
        # Extract path from probability map using thresholding and skeletonization
        path_binary = (path_prob_map > 0.5).astype(np.uint8)
        # Use skimage.morphology.skeletonize instead of scipy.ndimage.morphology.skeletonize
        path_skeleton = skeletonize(path_binary)
        
        # Extract path coordinates
        path_coords = np.where(path_skeleton)
        path = list(zip(path_coords[0], path_coords[1]))
        
        # Sort path from start to goal
        path = self._sort_path(path, start, end)
        
        return path
    
    def _sort_path(self, path, start, end):
        """Sort path points from start to end"""
        if not path:
            return []
        
        # Convert to list of tuples if it's not already
        path = [tuple(p) for p in path]
        
        # Start with the start point
        sorted_path = [start]
        remaining = set(path)
        
        # Iteratively find the closest point
        current = start
        while remaining and current != end:
            closest = min(remaining, key=lambda p: ((p[0]-current[0])**2 + (p[1]-current[1])**2))
            sorted_path.append(closest)
            remaining.remove(closest)
            current = closest
            
            # If we're close to the end, add it and break
            if ((current[0]-end[0])**2 + (current[1]-end[1])**2) < 25:  # threshold distance
                break
        
        # Add the end point
        if sorted_path[-1] != end:
            sorted_path.append(end)
            
        return sorted_path
    
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