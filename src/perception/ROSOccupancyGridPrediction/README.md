# ROS Occupancy Grid Prediction

This package contains ROS C++ Occupancy Grid Prediction framework which includes point cloud preprocessing, ground segementation, occupancy grid generation, and occupancy grid prediction. The pipeline follows the approach defined by Itkina et al. [1]. The package is compatible with models trained in Tensorflow and PyTorch provided as protocol buffers (.pb) and torch script (.pt), respectively. It contains PredNet [1], PredNet with TAAConvLSTM, and PredNet with SAAConvLSTM [2] models trained on the KITTI dataset [3]. Pointcloud can be provided in the form of a rosbag or directly from the robot Lidar sensors. This package does not contain the Tensorflow C++ API and LibTorch API (PyTorch C++), and the rosbags. The package is fully compatible with [Ford AV Dataset](https://avdata.ford.com/) [4].

## ROS Lidar pointcloud compatibility

Frame and topics names are compatible with Ford AV Dataset. To ensure the compatibility with other rosbag, the raw Lidar pointcloud topics needs to be renamed in the aggregate_points.cpp to match the topics and the number of Lidar sensors in the rosbag. The type of message used to represent the Lidar pointcloud and all other messages are defined in lidar_msgs. 

## How to run it?

1. Install Ford AV dataset ROS package and all dependencies from [Ford AV Dataset](https://avdata.ford.com/) [4] in your catkin_workspace.

2. Replace the multi_lidar_convert.launch in ford_demo/launch with the file provided in other. 

3. Provide the path to the prediction model in inference_torch.cpp (or inference_tf.cpp). Example models are provided in the lidar_pkg/models. If tensorflow models is used, also define the name of the input and output layer in the following lines.

4. In the same terminal, launch the node. Pick the right launch file depending if you are doing inference using Tensorflow or LibTorch.

  ```bash
  roslaunch lidar_pkg launch_all.launch 
  ```  
5. In a different terminal, start a rosbag from AV Dataset. Ensure that the naming convention of the topics is correct.
  
  ```bash
  rosbag play --clock path/to/the/rosbag
  ```
6. In the terminal with rosbag play command, press SPACE to stop/resume the rosbag. To see the predictions, press SPACE (pause). Predictions will appear to the right of the occupancy grid.

## Example of the rviz visualization

![](images/ROS_demo.gif)

## Experiments 

Tested on:
- Ubuntu 18.04
- ROS Melodic 
- Tensorflow r2.3 for CUDA 11.0 (compiled from source and linked accordingly in the CMake files, follow [official Tensorflow instructions](https://www.tensorflow.org/install/source))
- LibTorch 1.8.0 for CUDA 11.0 (binary downloaded from [PyTorch website](https://pytorch.org/cppdocs/installing.html))
- CUDA 11.0
- Nvidia RTX 2080Ti
- Intel i9-9900KF

Performance:

| Models        | Publish rate using Tensorflow (Hz) |  Publish rate using LibTorch (Hz)   |
| ------------- |:-------------:| -----:|
| PredNet    | 9.7 ± 0.52 | 13.5 ± 0.34 |
| PredNet with TAAConvLSTM      | -      |   6.5 ± 0.49 |
| PredNet with SAAConvLSTM | -      |    5.1 ± 0.45 |

## Acknowledgements 
The authors would like to acknowledge this project being made possible by the funding from the Ford-Stanford Alliance. The authors thank Stanford Dynamic Design Lab for providing Lidar processing stack.

Maintained by [Bernard Lange](https://web.stanford.edu/~blange/)

## References

[1] Itkina, M., Driggs-Campbell, K. and Kochenderfer, M.J., 2019, Dynamic Environment Prediction in Urban Scenes using Recurrent Representation Learning. In IEEE Intelligent Transportation Systems Conference (ITSC), pp. 2052-2059.

[2] Lange, B., Itkina, M. and Kochenderfer, M.J., 2020. Attention Augmented ConvLSTM for Environment Prediction. arXiv preprint arXiv:2010.09662.

[3] Geiger, A., Lenz, P., Stiller, C. and Urtasun, R., 2013. Vision meets robotics: The kitti dataset. The International Journal of Robotics Research, 32(11), pp. 1231-1237.

[4] Agarwal, S., Vora, A., Pandey, G., Williams, W., Kourous, H. and McBride, J., 2020. Ford Multi-AV Seasonal Dataset. arXiv preprint arXiv:2003.07969.



