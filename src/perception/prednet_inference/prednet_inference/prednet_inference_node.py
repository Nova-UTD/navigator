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
import numpy as np
from rclpy.node import Node
import time
import copy

from threading import Thread
from array import array as Array


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

from nav_msgs.msg import OccupancyGrid
from navigator_msgs.msg import Masses
from navigator_msgs.msg import Egma


import torch


import matplotlib.pyplot as plt

# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line

class PredNetNode(Node):

    def __init__(self):
        super().__init__('prednet_inference_node')

        # Subcribes to masses
        self.masses_sub = self.create_subscription(
            Masses, '/grid/masses', self.masses_callback, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.makePrediction)

        # Instantiates publisher
        self.pred_all_pub = self.create_publisher(
            Egma, '/grid/predictions', 10)
        self.prednet_combined_pub = self.create_publisher(
            OccupancyGrid, '/grid/predictions_combined', 10)

        # Stores the past 0.5 seconds of occupancy grids
        self.history_m = []

        # Message to be published
        self.prediction_msg = Egma()

        # Whether the required history has been obtained
        self.data_acquired = False

        # Amount of grids that have been made
        self.time = 0

        self.frameRate = 10

        # Time for when to accept the next grid
        self.lastAccepted = 0
        
        # Size of the grids
        self.sizeX = None
        self.sizeY = None

        # GPU that the prednet model will use
        self.device = 'cuda:0'
        
        # Whether the images have been made (for testing)
        self.madeExample = False
        
        # Model directry
        # Importing models
        modelDir = "/navigator/data/perception/models/prednet.pt"

        # Sets the device to the rigth GPU
        torch.cuda.set_device(torch.device(self.device))
        torch.cuda.empty_cache()
        
        # Prints out the GPU being used
        self.get_logger().info("Current GPU device: " + str(torch.cuda.current_device()))

        # Checks if Cuda is avaiable
        if torch.cuda.is_available():
            self.get_logger().info("CUDA available! Inference on GPU.")
        else:
            self.get_logger().info("Inference on CPU.")

        # Loads the prednet model
        try:
            self.prednet_model = torch.jit.load(modelDir, map_location=torch.device(self.device))
        except: 
            raise Exception("Couldn't load prednet model")

        # Makes sures the model is on the GPU
        try:
            self.prednet_model.to(torch.device(self.device))
        except:
            raise Exception("Couldn't load cuda")

        # Evaluates the model to make sure it's working correctly
        try:
            self.prednet_model.eval()
        except:
            raise Exception("Couldn't eval model")

    # Updates the clock for the header
    def clock_cb(self, msg):
        self.clock = msg.clock

    # Adds masses to history
    def masses_callback(self, mass):
        if self.clock.sec + 1e-9 * self.clock.nanosec - self.lastAccepted < 1. / self.frameRate  - 0.01:
            return
        

        # self.get_logger().info(str(self.clock.sec + 1e-9 * self.clock.nanosec))
        # Sets the grid size to the given grid size
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
                self.history_m[i] = self.history_m[i + 1]

            self.history_m[4] = mass

            # All the requrired data to make a prediction
            self.data_acquired = True
        
        #Updates last accepted time
        self.lastAccepted = self.clock.sec + 1e-9 * self.clock.nanosec
        
                
    
    def makePrediction(self):
        # 0.09s average prediction time

        # If there isn't enough past data to make a prediction
        if not self.data_acquired:
            return

        # To check how long it takes to make a prediction
        startTime = self.get_clock().now()

        # Creates tensor frames onto the GPU
        tensorFrames = self.createTensorFromFrames()

        # *** For Testing ***
        # Saves the image of the current occupancy grid
        if not self.madeExample and self.time > 100:
            IMAGES_FN = '/navigator/src/perception/prednet_inference/predictionTest/'

            fig1 = plt.figure()
            ax1 = fig1.add_subplot(111)
            displayOutput = (
                tensorFrames[0, 4, 0, :, :] * 0.5 + (tensorFrames[0, 4, 1, :, :] * -0.5 + 0.5)) * 100
            displayOutput = displayOutput.cpu().detach().numpy()
            h = ax1.matshow(displayOutput)
            plt.colorbar(h)
            fig1.savefig(IMAGES_FN + 'original_5' + '.png')
            plt.close()

        # Makes the prediction and publishes the prediction
        prediction = self.Infer(tensorFrames)
        self.Publish(prediction)

        # *** For Testing ***
        # Saves the images of the predicted occupancy grids
        if not self.madeExample and self.time > 100:
            self.get_logger().info("Making visualization")
            self.madeExample = True
            IMAGES_FN = '/navigator/src/perception/prednet_inference/predictionTest/'

            displayOutput = torch.empty(20, self.sizeX, self.sizeY)
            for i in range(prediction.shape[1]):
                displayOutput[i] = (
                    prediction[0, i, 0, :, :] * 0.5 + (prediction[0, i, 1, :, :] * -0.5 + 0.5)) * 100
            # displayOutput = (prediction[0,:,0, :, :] * 0.5 + (prediction[0,:,1, :, :] * -0.5 + 0.5)) * 100
            displayOutput = displayOutput.cpu().detach().numpy()

            for i in range(displayOutput.shape[0]):
                fig1 = plt.figure()
                ax1 = fig1.add_subplot(111)
                h = ax1.matshow(displayOutput[i])
                plt.colorbar(h)
                # plt.show()[]
                fig1.savefig(IMAGES_FN + 'prediction_example_' +
                             str(i) + '.png')
                plt.close()

        # Gets the total time takes to make the prediction
        totalTimeTaken = self.get_clock().now() - startTime

        # self.get_logger().info("Published the model prednet Model Prediction in " + str(totalTimeTaken.nanoseconds / pow(10,9)))

    def createTensorFromFrames(self):
        # Creates the tensor object
        input_data = torch.empty(
            (1, 20, 2, self.sizeX, self.sizeY), device=torch.device(self.device))

        # Gets the 5 known past masses
        for i in range(5):
            # Puts the masses into the tensor
            input_data[0, i, 0] = torch.tensor(
                np.array(self.history_m[i].occ).reshape((self.sizeX, self.sizeY)))
            input_data[0, i, 1] = torch.tensor(
                np.array(self.history_m[i].free).reshape((self.sizeX, self.sizeY)))

        return input_data

    def Infer(self, input):
        # Makes the prediction using the prednet model
        output = self.prednet_model(input)

        return output

    def Publish(self, output):
        # Prediction message instantiation
        self.prediction_msg = Egma()

        # Moves the output off the GPU onto the CPU
        new_output = output.to("cpu")
        # 5+ indexs are the predicted images
        new_output = new_output[:, 5:, :, :, :]

        # Gets the clock for the headers
        cur_clock = self.clock

        probability_grids = []

        for i in range(new_output.shape[1]):

            # Makes a new clock for time that the occupancy grid is predicted for
            cur_clock = copy.deepcopy(cur_clock)
            cur_clock.nanosec = cur_clock.nanosec + pow(10, 7)
            if cur_clock.nanosec >= pow(10, 9):
                cur_clock.nanosec = cur_clock.nanosec - pow(10, 9)
                cur_clock.sec = cur_clock.sec + 1

            # Creates the occupancy grid message
            occ_grid_msg = OccupancyGrid()
            occ_grid_msg.header.stamp = cur_clock
            occ_grid_msg.header.frame_id = "base_link"

            occ_grid_msg.info.resolution = 1. / 3.
            occ_grid_msg.info.width = self.sizeX
            occ_grid_msg.info.height = self.sizeY

            # Calcualtes the probabilitisc occupancy grid
            prob_2D = (new_output[0, i, 0] * 0.5 +
                       (new_output[0, i, 1] * -0.5 + 0.5)) * 100
            probability_grids.append(prob_2D.detach().numpy())

            # Flattens the grid and puts it into the message
            occ_grid_msg.data = prob_2D.flatten().type(torch.int8).numpy().tolist()

            output_occ = new_output

            # Adds the occ grid message to the prediction message
            self.prediction_msg.egma.append(occ_grid_msg)

        # aggregate_probability = np.zeros((128, 128))
        # for grid in probability_grids:
        #     aggregate_probability += grid

        # Only consider the last timeframe
        aggregate_probability = probability_grids[-1]

        aggregate_probability /= len(probability_grids)

        aggregate_probability = aggregate_probability.T  # Flip rows and columns
        # plt.imshow(aggregate_probability)
        # plt.show()

        # Publish the prediction message
        self.pred_all_pub.publish(self.prediction_msg)

        # Publish combined (flattened) message
        combined_grid = OccupancyGrid()
        combined_grid.info = self.prediction_msg.egma[0].info
        combined_grid.data = Array(
            'b', aggregate_probability.ravel().astype(np.int8))
        combined_grid.header.stamp = self.clock
        combined_grid.header.frame_id = "base_link"
        combined_grid.info.origin.position.z = 0.2
        combined_grid.info.origin.position.x = -64.0 * (1. / 3.)
        combined_grid.info.origin.position.y = -64.0 * (1. / 3.)
        self.prednet_combined_pub.publish(combined_grid)


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
