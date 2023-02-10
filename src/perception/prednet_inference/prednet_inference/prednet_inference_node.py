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
# https://1drv.ms/v/s!An_6l1bJUIotjlN-qg-uqKdIORgC?e=s5RDv6

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time
import copy

from threading import Thread


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2 

from nav_msgs.msg import OccupancyGrid
from nova_msgs.msg import Masses
from nova_msgs.msg import Egma


import torch


import matplotlib.pyplot as plt


class PredNetNode(Node):

    def __init__(self):
        super().__init__('prednet_inference_node')
        
        self.masses_sub = self.create_subscription(
            Masses, '/grid/masses', self.masses_callback, 10)
        
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        
        self.timer = self.create_timer(0.1, self.makePrediction)
        
        self.pred_all_pub = self.create_publisher(
           Egma, '/grid/predictions', 10)
        
        self.history_m = []
        
        self.prediction_msg = Egma()
        
        self.data_acquired = False
        
        self.time = 0
        
        self.sizeX = None
        self.sizeY = None
        
        self.predicting = False
        
        self.device = 'cuda:2'
        
        self.madeExample = False
        
        # Importing models
        modelDir = "/navigator/data/perception/models/prednet.pt"
        
        torch.cuda.set_device(torch.device(self.device))
        # torch.cuda.empty_cache()
        
        self.get_logger().info("Current GPU device: " + str(torch.cuda.current_device()))
        
        
        if torch.cuda.is_available():
            self.get_logger().info("CUDA available! Inference on GPU.")
        else:
            self.get_logger().info("Inference on CPU.");  
        
        try:
            self.prednet_model = torch.jit.load(modelDir, map_location=torch.device(self.device))
        except:
            raise Exception("Couldn't load prednet model")
        
        try:
            self.prednet_model.to(torch.device(self.device))
        except:
            raise Exception("Couldn't load cuda")
        
        try:
            self.prednet_model.eval()
        except:
            raise Exception("Couldn't eval model")
        
    def clock_cb(self, msg):
        
        self.clock = msg.clock
    
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
        
                
    
    def makePrediction(self):
        # 0.09 average prediction time
        if not self.data_acquired:
            return
        
        startTime = self.get_clock().now()
                
        tensorFrames = self.createTensorFromFrames()
        
        if  not self.madeExample and self.time > 100:
            IMAGES_FN = '/navigator/src/perception/prednet_inference/predictionTest/'
            
            fig1 = plt.figure()
            ax1 = fig1.add_subplot(111)
            displayOutput = (tensorFrames[0,4,0, :, :] * 0.5 + (tensorFrames[0,4,1, :, :] * -0.5 + 0.5)) * 100
            displayOutput = displayOutput.cpu().detach().numpy()
            h = ax1.matshow(displayOutput)
            plt.colorbar(h)
            fig1.savefig(IMAGES_FN + 'original_5' + '.png') 
            plt.close()
        
        prediction = self.Infer(tensorFrames)
        self.Publish(prediction)
        
        if not self.madeExample and self.time > 100:
            self.get_logger().info("Making visualization")
            self.madeExample = True
            IMAGES_FN = '/navigator/src/perception/prednet_inference/predictionTest/'
            
            displayOutput =  torch.empty(20,self.sizeX, self.sizeY)
            for i in range(prediction.shape[1]):
                displayOutput[i] = (prediction[0,i,0, :, :] * 0.5 + (prediction[0,i,1, :, :] * -0.5 + 0.5)) * 100
            # displayOutput = (prediction[0,:,0, :, :] * 0.5 + (prediction[0,:,1, :, :] * -0.5 + 0.5)) * 100
            displayOutput = displayOutput.cpu().detach().numpy()
            
            for i in range(displayOutput.shape[0]):
                fig1 = plt.figure()
                ax1 = fig1.add_subplot(111)
                h = ax1.matshow(displayOutput[i])
                plt.colorbar(h)
                    #plt.show()[]
                fig1.savefig(IMAGES_FN + 'prediction_example_' + str(i) + '.png') 
                plt.close()
            
                
                
        totalTimeTaken = self.get_clock().now() - startTime
        
        # self.get_logger().info("Published the model prednet Model Prediction in " + str(totalTimeTaken.nanoseconds / pow(10,9)))
                
    
    def createTensorFromFrames(self):
        # Creates the tensor object
        input_data = torch.empty((1, 20, 2, self.sizeX, self.sizeY), device=torch.device(self.device))
        
        # Gets the 5 known past masses
        for i in range(5):
            # Puts the masses into the tensor
            input_data[0,i,0] = torch.tensor(np.array(self.history_m[i].occ).reshape((self.sizeX, self.sizeY)))
            input_data[0,i,1] = torch.tensor(np.array(self.history_m[i].free).reshape((self.sizeX, self.sizeY)))
        
        return input_data
    
    def Infer(self, input):
        output = self.prednet_model(input)
        
        return output
    
    def Publish(self, output):
        
        self.prediction_msg = Egma()
        
        
        new_output = output.to("cpu")
        new_output = new_output[:,5:,:,:,:]
        
        cur_clock = self.clock
        
        for i in range(new_output.shape[1]):
            cur_clock = copy.deepcopy(cur_clock)
            
            cur_clock.nanosec = cur_clock.nanosec + pow(10, 7)
            
            if cur_clock.nanosec >= pow(10, 9):
                cur_clock.nanosec = cur_clock.nanosec - pow(10, 9)
                cur_clock.sec = cur_clock.sec + 1
            
            occ_grid_msg = OccupancyGrid()
            occ_grid_msg.header.stamp = cur_clock
            occ_grid_msg.header.frame_id = "base_link"
            
            occ_grid_msg.info.resolution = 1. / 3.
            occ_grid_msg.info.width = self.sizeX
            occ_grid_msg.info.height = self.sizeY
            
            prob_2D = (new_output[0,i,0] * 0.5 + (new_output[0,i,1] * -0.5 + 0.5)) * 100
            
            occ_grid_msg.data = prob_2D.flatten().type(torch.int8).numpy().tolist()
            
            output_occ = new_output
            
            self.prediction_msg.egma.append(occ_grid_msg)
        
        
        
        self.pred_all_pub.publish(self.prediction_msg)



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
