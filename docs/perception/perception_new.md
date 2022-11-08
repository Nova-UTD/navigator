---
layout: default
title: Perception and Prediction Design Document (draft)
---

# Perception and Prediction Design Document (draft)
{: .no_toc }

*Maintained by Ashwin*
## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---






## System Overview 

 

### Current System: 

 

#### Prediction: 

    - Zones in place of dynamic object detection 

    - Acts as a safety net cast over objects in view of the vehicle 

    - Car "reacts" to zone/wall proximity and zone speed argument 

    - Slows down to minimum speed argument, regardless if there are multiple zones 

    - Primitive 

    - Unable to further expand implementation 

    - Zone algorithm highlighted more in depth in behavior planning and control document 

 

### Planned System: 

#### Perception: 

Phase 1: 

- 3D/2D bounding boxes (tracked) 

- Static occupancy grid 

Phase 2: 

- Landmark detection (Stop lights priority) 

- Semantic encoding 

- Map masks: Stop lines, Cross walks 
 

Phase 3: 

- Tune models within simulator 

- Eventually deploy and test on vehicle 

 

#### Prediction: 

Phase 1: 

- Physics-based prediction implemented 

- Treats cars like particles (acceleration & velocity) 

- Dynamic occupancy grid 

Phase 2: 

- ML-based occupancy grid (from physics-based) 

- Non-sequential learning 

- Sequential learning 

- Recurrent Neural Nets 

Dynamic Bayesian Network 

Phase 3: 

- ML refined 

- Map priors 

- Deployed and tested on vehicle 

 

******** Insert first image ****

![perception_flowchart](/navigator/assets/res/perception_flow.jpeg)
 

 

### Proposed structures for Phase 1: 

 

#### 3D/2D bounding boxes: 

3D: 

Estimating 3D bounding boxes based on 2D images: 

A method for 3D object detection using 2D images. This method involves utilizing a deep convolutional network to obtain relatively stable 3D object properties, and utilize this along with some geometric constraints provided by a 2d bounding box (would probably need to figure this out using something like YOLOv3) in order to construct a 3D bounding box for the corresponding object.  

Given the pose of the object in the camera coordinate frame (R, T) ∈ SE(3) and the camera intrinsic matrix K, the projection of a 3D point Xo = [X, Y, Z, 1]^T in the object’s coordinate frame into the image x = [x, y, 1]T is:  
                                                              x = K [R T] Xo    [1] 

 

 

2D: 

2D object detection from a BEV representation (Bird's Eye View, looking down from the Z axis) of our pseudo-lidar point cloud data may be more efficient than processing a generic 3D convolution. This architecture may also include semantic segmentation, effectively enabling us to accomplish landmark detection as well without any additional computation. 

Can be a solution for stereo-images. 

Depth correction w/ existing LiDAR sensors 

2D BEVDetNet Object Detection 

Pseudo-LiDAR 

 

 

 

#### System overview for Dynamic Environment Prediction 

Our system uses point clouds for dynamic environment prediction. We use a ConvLSTM architecture intended for video frame prediction to instead predict the local environment surrounding an autonomous agent across future time steps. For this purpose, we adapt the PredNet architecture designed for video frame prediction in autonomous driving scenes. The ConvLSTM is expected to learn an internal representation of the dynamics within the local environment from occupancy grid data. The grid inputs are generated from LiDAR measurement recordings in the KITTI dataset taken across a variety of urban scenes [1].  

  

#### Proposed structure for Dynamic Environment Prediction 

![dep_flow](/navigator/assets/res/perception_flow_prednet.png)

  

Inputs for Dynamic Environment Prediction 

- Point Cloud 

Produces for Dynamic Environment Prediction 

- Ground Segmentation 

- Occupancy grid and DOGMA 

- PredNet (Neural Network Architecture) 


Dynamic Environment Prediction does not handle...  

- More research needed… 

  

Dynamic Environment Prediction: Ground Segmentation 

Prior to generating an occupancy grid, the ground must be segmented and removed from the LiDAR point cloud.  Markov Random Field (MRF) algorithm that exploits local spatial relationships, avoiding the assumption of a planar ground [2][1]. 

  

Dynamic Environment Prediction: Dynamic Occupancy Grid Maps (DOGMas) 

A DOGMa is an evidential grid containing both occupancy and dynamic state information (e.g., velocity). We generate DOGMas via the procedure outlined by Nuss et al. [4]. There are two parallel processes that occur: occupancy grid updates and cell-wise velocity estimates [1]. 

  

Occupancy Grids 

We consider DST-based occupancy grids computed from LiDAR measurements as detailed by Nuss et al. [4]. DST deals with a set of exhaustive hypotheses formed from a frame of discernment and the associated belief masses. In the case of an occupancy grid, our frame of discernment is: Ω = {F, O}, where F is free space and O is occupied space. Thus, the set of hypotheses is: \{∅, \{F \}, \{O\}, \{F, O\}\}. The null set ∅ is impossible in this context as a cell physically cannot be neither occupied nor unoccupied. Thus, our exhaustive set of hypotheses is: \{\{F \}, \{O\}, \{F, O\}\}. The sum of the belief masses over the possible hypotheses for each individual cell must equal one by definition, akin to probabilities [1]. 
  
 
 

Velocity Estimation 

To incorporate dynamics, we estimate the velocity for each cell. The velocity estimates use a DST approximation for a probability hypothesis density filter with multi-instance Bernoulli (PHD/MIB) [5]. DST allows for the particle filter to run more efficiently by initializing particles only in cells with occupied masses above a specified threshold, avoiding occluded regions without measurements[1]. 
 

  

Dynamic Environment Prediction: Neural Network Architecture  

We repurpose the PredNet architecture to learn the spatial and temporal representation of the environment by training it on occupancy grids instead of images. The convolutional layers exploit contextual information to correlate the occupied cells, removing the cell independence assumption [6]. The self-supervised nature of sequential data prediction is advantageous as human-labeled LiDAR data is expensive to obtain [6]. In this framework, the labels are simply the input environment representation (grids) at a later time instance. Although the original PredNet model was designed for video data, we demonstrate that the architecture can be re-used in the LiDAR setting [1]. 

  

Dynamic Environment Prediction: Experiments 

- Dataset Generation 

   - LiDAR Measurement Grids 

    - The KITTI HDL-64E Velodyne LiDAR dataset was augmented for use in occu- pancy grid prediction [7]. The dataset contains a variety of urban road scenes in Karlsruhe, Germany. We use 35, 417 frames (138 driving sequences) for training, 496 frames (3 driving sequences) for validation, and 2, 024 frames (7 driving sequences) for testing. Velodyne LiDAR point cloud measurements are obtained at 10 Hz.  
        
    - Each LiDAR point cloud is filtered to remove the points corresponding to the ground as described in Section II. Then, a simple form of ray-tracing is performed to determine the free space between a LiDAR measurement and the ego vehicle. Each resulting local grid is centered at the ego vehicle GPS coordinate position. The shorter grid range is acceptable for slower speeds in urban settings, as is the case in the KITTI dataset [7][1]. 
 

    - Dynamic Occupancy Grid Maps 

        - Dynamic Occupancy Grid Maps: The DOGMa’s occupancy and velocity information is computed from the LiDAR data as outlined in Section II. The velocities are then filtered to remove static measurements according to the cell-wise Mahalanobis distance: τ = vT P v, where v is the velocity vector and P is the cell’s covariance matrix as computed from the particles [21]. Cells with occupancy masses below a threshold are also removed. The velocities are then normalized to the range [−1, 1] and stacked with either (1) the pignistic probability (Eq. (5)) or (2) the DST mass (Eq. (2) and Eq. (3)) occupancy grids, forming the input to the network. [1] 
    
 

- PredNet Experiments 

    - PredNet was trained and tested on an NVIDIA GeForce GTX 1070 GPU. At test time, one sequence (15 predictions totaling 1.5 s ahead) took on average 0.1 s to run. [1] 

 