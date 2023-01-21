#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>
#include "inference_torch.h"
#include <torch/script.h> // One-stop header.

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <torch/torch.h>

#include <nav_msgs/OccupancyGrid.h>
#include <lidar_msgs/masses.h>
#include <lidar_msgs/prediction.h>

PLUGINLIB_EXPORT_CLASS(lidar_pkg::InferenceTorch, nodelet::Nodelet)

namespace lidar_pkg
{
  void InferenceTorch::onInit()
  {
    ROS_INFO("Initialized PyTorch Inference.");
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();

    // Path to the model
    std::string jitPath = "/home/bernard/catkin_ws/src/ROS_OGP/RealTimeEnvironmentPrediction/models/taa.pt";

    // Loading the model
    ROS_INFO("Loading the module...");
    torch::jit::script::Module module;
    try {
       module = torch::jit::load(jitPath);
     }
     catch (const c10::Error& e) {
       std::cerr << "error loading the model\n";
       return -1;
     }

    ROS_INFO("Loaded!")

    pred_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("prediction", 1);
    pred_all_pub = private_nh.advertise<lidar_msgs::prediction>("prediction_all", 1);
    occupancy_grid_sub = private_nh.subscribe("occupancy", 1, &Inference::occupancy_grid_callback, this);
    masses_sub = private_nh.subscribe("masses", 1, &Inference::masses_callback, this);
    timer = private_nh.createTimer(ros::Duration(0.02), &Inference::timerCallback, this);
  }

  void Inference::timerCallback(const ros::TimerEvent&)
  {
    if (data_acquired==true)
    {
      ROS_INFO("Inside the timer");

      //DEFINE APPROPRAITE GLOBAL VARIABLES

      tensorflow::Tensor input(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,20,128,128,2}));
      std::vector<tensorflow::Tensor> output;

      // Create/Fill in a tensor.

      input = createTensorFromFrames();

      // Make a prediction

      createPredictionAndPublish(input);
      //saveTensor(output);
    }
    else {
      ROS_INFO("Clock is paused...");
    }
  }

  void Inference::createPredictionAndPublish(input)
  {
    ROS_INFO("Publishing");
    prediction_msg.prediction.clear();
    at::Tensor output = module.forward(inputs).toTensor();
    int grid_size = 128;
    auto output_values = output[0].tensor<float, 5>();
    int probability;
    
    for (unsigned int t = 0; t < 20; t++){
      occupancy_msg.data.clear();
      occupancy_msg.info.resolution =  1./3.;
      occupancy_msg.info.width = grid_size;
      occupancy_msg.info.height = grid_size;
      for (unsigned int i = 0; i < grid_size; i++){
        for (unsigned int j = 0; j < grid_size; j++){
          //Modify this accordingly
          probability = (int)(100*(0.5*output_values(0,t,i,j,0)+0.5*(1.0 - output_values(0,t,i,j,1))));
          occupancy_msg.data.push_back(probability);
        }
      }
      prediction_msg.prediction.push_back(occupancy_msg);
    }
    pred_all_pub.publish(prediction_msg);
    pred_pub.publish(occupancy_msg);
  }

  void Inference::saveTensor(std::vector<tensorflow::Tensor>& output)
  {
    // auto output_values = output[0].tensor<float, 5>();
    // for(unsigned int t=0;t<20;t++){
    //   std::vector<float> occ;
    //   std::vector<float> free;
    //
    //   for(unsigned int x=0;x<128;x++){
    //     for(unsigned int y=0;y<128;y++){
    //       occ.push_back(output_values(0,t,x,y,0));
    //       free.push_back(output_values(0,t,x,y,1));
    //     }
    //   }
    //   std::ofstream outfile_occ("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/prediction_occ"+std::to_string(t)+".txt");
    //   for (const auto &e : occ) outfile_occ << e << "\n";
    //
    //   std::ofstream outfile_free("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/prediction_free"+std::to_string(t)+".txt");
    //   for (const auto &e : free) outfile_free << e << "\n";
    //
    // }
  }

  void Inference::createTensorFromFrames()
  {
    std::vector<torch::jit::IValue> input;


    // MODIFY
    using namespace::tensorflow::ops;
    bool past = true;
    auto input_map = input.tensor<float,5>();
    lidar_msgs::masses data;

    for(unsigned int t = 0; t<20;t++){
      if (t<5){
        data = history_m[t];
      }
      else {
        past = false;
      }
      int count = 0;
      for(unsigned int x = 0; x<128;x++){
        for(unsigned int y = 0; y<128;y++){
          if(past==true){
            input_map(0,t,x,y,0) = (float)data.occ[count]; //check the ordering + add depth.
            input_map(0,t,x,y,1) = (float)data.free[count];
          }
          count++;
        }
      }
    }
    return input
  }

  void Inference::saveFrame(lidar_msgs::masses& data, int t)
  {
    // std::vector<float> occ;
    // occ = data.occ;
    // std::vector<float> free;
    // free = data.free;
    // std::ofstream outfile_occ("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/past_occ"+std::to_string(t)+".txt");
    // for (const auto &e : occ) outfile_occ << e << "\n";
    //
    // std::ofstream outfile_free("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/past_free"+std::to_string(t)+".txt");
    // for (const auto &e : free) outfile_free << e << "\n";

  }

  void Inference::occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy)
  {
    ROS_INFO("Data collected");
    time++;
    nav_msgs::OccupancyGrid zeross;
    //zeross.data = occupancy.data;
    if (time <= 5){
      history_n.push_back(*occupancy);
    }
    else {
      data_acquired = true;
      for (unsigned int i=0; i<=3; i++){
        history_n[i] = history_n[i+1];
      }
    history_n[4]=(*occupancy);
    }
      ROS_INFO("Done with Data collected");
  }

  void Inference::masses_callback(const lidar_msgs::masses::ConstPtr& masses_data)
  {
    ROS_INFO("Data collected Masses");
    //time++;
    nav_msgs::OccupancyGrid zeross;
    //zeross.data = occupancy.data;
    if (time <= 5){
      history_m.push_back(*masses_data);
    }
    else {
      data_acquired = true;
      for (unsigned int i=0; i<=3; i++){
        history_m[i] = history_m[i+1];
      }
    history_m[4] = (*masses_data);
    }
      ROS_INFO("Done with Data collected");
  }
}
