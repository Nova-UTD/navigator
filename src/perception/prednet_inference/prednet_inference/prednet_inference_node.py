'''
Package: sensor_processing
   File: lidar_processing_node.py
 Author: Will Heitman (w at heit dot mn)

Node to filter and process raw LiDAR pointclouds

This node specifically deals with quirks with the
CARLA simulator. Namely, synching issues mean that
raw LiDAR streams "flicker" from left-sided PCDs
to right-sided ones. This node first merges
these left- and right-sided PCDs into a complete
cloud before cutting out points near the car.
'''

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time


# Message definitions
from nav_msgs.msg import OccupancyGrid

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32

from nova_msgs.msg import Masses
from nova_msgs.msg import Prediction

import torch


import matplotlib.pyplot as plt


class PredNetNode(Node):

    def __init__(self):
        super().__init__('lidar_processing_node')
        self.masses_sub = self.create_subscription(
            Masses, '/', self.semantic_lidar_cb, 10)

        self.pred_all_pub = self.create_publisher(
            Prediction, '/', 10)
        
        self.history_m = []
        
        self.prediction_msg = Prediction()
        
        self.data_acquired = False
        
        self.time = 0
        
        self.sizeX = None
        self.sizeY = None
    
    def masses_callback(self, mass):
        
        self.sizeX = mass.width
        self.sizeY = mass.height
        
        # Iterates Time
        self.time += 1
        
        # If less than 5 previous masses have been read then append to the back of the array
        if self.time <= 5:
            self.history_m.append(mass)
        # Else Shift the array backwards and add the new mass to the 4th index
        else:
            for i in range(4):
                self.history_m[i] = self.history_m[i+ 1]
            
            self.history_m[4] = mass
            
            # All the requrired data to make a prediction
            self.data_acquired = True
    
    def createTensorFromFrames(self):
        # Creates the tensor object
        input_data = torch.empty({1, 20, 2, self.sizeX, self.sizeY})
        
        # Gets the 5 known past masses
        for i in range(5):
            # Puts the masses into the tensor
            input_data[1,i,0] = torch.tensor(np.array(self.history_m[i].occ).reshape((self.sizeX, self.sizeY)))
            input_data[1,i,1] = torch.tensor(np.array(self.history_m[i].free).reshape((self.sizeX, self.sizeY)))
        
        return input_data
    
    def Infer(self, input):
        return
    
    def Publish(self, output):
        return
    
    
        
        
    


def main(args=None):
    rclpy.init(args=args)

    prednet_node = PredNetNode()

    rclpy.spin(prednet_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    prednet_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
