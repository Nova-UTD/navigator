#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>
#include "inference_tf.h"
#include "utils.h"

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "tensorflow/core/public/session.h"
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/array_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#include "tensorflow/cc/framework/ops.h"
#include <nav_msgs/OccupancyGrid.h>
#include <lidar_msgs/masses.h>
#include <lidar_msgs/prediction.h>

PLUGINLIB_EXPORT_CLASS(lidar_pkg::InferenceTF, nodelet::Nodelet)

namespace lidar_pkg
{
  void InferenceTF::onInit()
  {
    ROS_INFO("Initialized InferenceTF.");
    tensorflow::Tensor input(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,20,128,128,2}));
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
    // Set dirs variables
    std::string graphPath = "/home/bernard/catkin_ws/src/ROS_OGP/ROSOccupancyGridPrediction/models/prednet_1.pb";
    std::string GRAPH = "prednet_1.pb";

    // Set input & output nodes names
    inputLayer = "input_1_1:0";
    outputLayer = "pred_net_1_1/transpose_1:0";

    // Load and initialize the model from .pb file
    ROS_INFO("Getting the graph...");
  //  std::string graphPath = tensorflow::io::JoinPath(ROOTDIR, GRAPH);
   ROS_INFO("Loading the graph...");
   tensorflow::Status loadGraphStatus = loadGraph(graphPath, &session);
    if (!loadGraphStatus.ok()) {
      ROS_INFO("Not loaded");
        //return 0;
    }
    else
    {
      ROS_INFO("Loaded");
    }

    pred_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("prediction", 1);
    pred_all_pub = private_nh.advertise<lidar_msgs::prediction>("prediction_all", 1);
    occupancy_grid_sub = private_nh.subscribe("occupancy", 1, &InferenceTF::occupancy_grid_callback, this);
    masses_sub = private_nh.subscribe("masses", 1, &InferenceTF::masses_callback, this);
    timer = private_nh.createTimer(ros::Duration(0.001), &InferenceTF::timerCallback, this);
  }

  void InferenceTF::timerCallback(const ros::TimerEvent&)
  {
    if (data_acquired==true)
    {
      ROS_INFO("Inside the timer");

      //DEFINE APPROPRAITE GLOBAL VARIABLES

      tensorflow::Tensor input(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,20,128,128,2}));
      std::vector<tensorflow::Tensor> output;

      // Create/Fill in a tensor.

      createTensorFromFrames(input);
      std::cout << "BEFORE THE SESSION RUNNING" << std::endl;
      output.clear();
      tensorflow::Status runStatus = session->Run({{inputLayer, input}}, {outputLayer}, {}, &output);
      std::cout << "AFTER THE SESSION RUNNING" << std::endl;

      if (!runStatus.ok()) {
          std::cout << "Running model failed: " << runStatus;
          //return 0;
      }
      else {
        ROS_INFO("Prediction made");
      }
      createPredictionAndPublish(output);
      //saveTensor(output);
    }
    else {
      ROS_INFO("Clock is paused not realy but yeah");
    }
  }

  void InferenceTF::createPredictionAndPublish(std::vector<tensorflow::Tensor>& output)
  {
    ROS_INFO("Publishing");
    int grid_size = 128;
    prediction_msg.prediction.clear();
    auto output_values = output[0].tensor<float, 5>();
    int probability;
    for (unsigned int t = 0; t < 20; t++){
      occupancy_msg.data.clear();
      occupancy_msg.info.resolution =  1./3.;
      occupancy_msg.info.width = grid_size;
      occupancy_msg.info.height = grid_size;
      for (unsigned int i = 0; i < grid_size; i++){
        for (unsigned int j = 0; j < grid_size; j++){
          probability = (int)(100*(0.5*output_values(0,t,i,j,0)+0.5*(1.0 - output_values(0,t,i,j,1))));
          occupancy_msg.data.push_back(probability);
        }
      }
      prediction_msg.prediction.push_back(occupancy_msg);
    }
    pred_all_pub.publish(prediction_msg);
    pred_pub.publish(occupancy_msg);
  }

  void InferenceTF::saveTensor(std::vector<tensorflow::Tensor>& output)
  {
    auto output_values = output[0].tensor<float, 5>();
    for(unsigned int t=0;t<20;t++){
      std::vector<float> occ;
      std::vector<float> free;

      for(unsigned int x=0;x<128;x++){
        for(unsigned int y=0;y<128;y++){
          occ.push_back(output_values(0,t,x,y,0));
          free.push_back(output_values(0,t,x,y,1));
        }
      }
      std::ofstream outfile_occ("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/prediction_occ"+std::to_string(t)+".txt");
      for (const auto &e : occ) outfile_occ << e << "\n";

      std::ofstream outfile_free("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/prediction_free"+std::to_string(t)+".txt");
      for (const auto &e : free) outfile_free << e << "\n";

    }
  }

  void InferenceTF::createTensorFromFrames(tensorflow::Tensor& input)
  {
    using namespace::tensorflow::ops;
    tensorflow::Status status;
    auto root = tensorflow::Scope::NewRootScope();
    std::unique_ptr<tensorflow::Session> session(tensorflow::NewSession({}));
    tensorflow::GraphDef graph;
    bool past = true;
    auto input_map = input.tensor<float,5>();
    lidar_msgs::masses data;
    ROS_INFO("STARTED_SAVING");
    for(unsigned int t = 0; t<20;t++){
      if (t<5){
        data = history_m[t];
        //saveFrame(data,t);
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
          else{
            //input_map(0,t,x,y,0) = 0.0;
            //input_map(0,t,x,y,1) = 0.0;
          }
          count++;
        }
      }
    }
    //history_m.clear();
    //history_n.clear();
  }

  void InferenceTF::saveFrame(lidar_msgs::masses& data, int t)
  {
    std::vector<float> occ;
    occ = data.occ;
    std::vector<float> free;
    free = data.free;
    std::ofstream outfile_occ("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/past_occ"+std::to_string(t)+".txt");
    for (const auto &e : occ) outfile_occ << e << "\n";

    std::ofstream outfile_free("/home/ford/catkin_ws/src/lidar/lidar_pkg/src/data/past_free"+std::to_string(t)+".txt");
    for (const auto &e : free) outfile_free << e << "\n";

  }

  void InferenceTF::occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy)
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

  void InferenceTF::masses_callback(const lidar_msgs::masses::ConstPtr& masses_data)
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
