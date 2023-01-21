#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>
#include "inference_torch.h"

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <torch/torch.h>
#include <torch/script.h> // One-stop header.

#include <nav_msgs/OccupancyGrid.h>
#include <lidar_msgs/masses.h>
#include <lidar_msgs/prediction.h>

#include <memory>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(lidar_pkg::InferenceTorch, nodelet::Nodelet)

using namespace std::chrono;

namespace lidar_pkg
{
  void InferenceTorch::onInit()
  {
    ROS_INFO("Initialized PyTorch Inference.");

    if (torch::cuda::is_available())
    {
      ROS_INFO("CUDA available! Inference on GPU.");
      // device =  torch::Device(torch::kCUDA);
    }
    else
    {
      ROS_INFO("Inference on CPU.");
      // device =  torch::Device(torch::kCPU);
    }

    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();

    // Path to the model
    std::string jitPath = "/home/bernard/catkin_ws/src/ROS_OGP/ROSOccupancyGridPrediction/models/prednet_with_taa.pt";

    // Loading the model
    ROS_INFO("Loading the module...");
    try
    {
      module = std::make_shared<torch::jit::script::Module>(torch::jit::load(jitPath));
      module->to(torch::kCUDA);
      module->eval();
      torch::NoGradGuard no_grad;
      // torch::Tensor input_data = torch::ones({1, 20, 2, 128, 128});
      // std::vector<torch::jit::IValue> input;
      // input.push_back(input_data.to(torch::kCUDA));
      // at::Tensor output = module->forward(input).toTensor();

      // std::shared_ptr<torch::jit::script::Module> module = torch::jit::load(jitPath);
      // module->to(torch::kCUDA);
    }
    catch (const c10::Error &e)
    {
      std::cerr << "error loading the model\n";
      // Something for errors
    }

    ROS_INFO("Loaded!");

    pred_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("prediction", 1);
    pred_all_pub = private_nh.advertise<lidar_msgs::prediction>("prediction_all", 1);
    // occupancy_grid_sub = private_nh.subscribe("occupancy", 1, &InferenceTorch::occupancy_grid_callback, this);
    masses_sub = private_nh.subscribe("masses", 1, &InferenceTorch::masses_callback, this);
    timer = private_nh.createTimer(ros::Duration(0.001), &InferenceTorch::timerCallback, this); // Reduced time but it didn't change much
  }

  void InferenceTorch::timerCallback(const ros::TimerEvent &)
  {
    auto start = high_resolution_clock::now();
    std::vector<torch::jit::IValue> input;
    if (data_acquired == true)
    {
      // ROS_INFO("Processing masses");
      createTensorFromFrames(input);
    }
    else
    {
      // Warm-up for LibTorch. First inference is slow
      ROS_INFO("Not enough data...");
      torch::Tensor input_data = torch::ones({1, 20, 2, 128, 128});
      input.push_back(input_data.to(torch::kCUDA));
    }
    auto stop1 = high_resolution_clock::now();
    // ROS_INFO("Making predictions");
    torch::Tensor output = Infer(input);
    auto stop2 = high_resolution_clock::now();
    // ROS_INFO("Publishing the prediction");
    Publish(output);
    auto stop3 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start);
    auto duration2 = duration_cast<microseconds>(stop2 - stop1);
    auto duration3 = duration_cast<microseconds>(stop3 - stop2);
    // std::cout << "Times: " << duration1.count() << " " << duration2.count() << " " << duration3.count() << std::endl;
  }

  torch::Tensor InferenceTorch::Infer(const std::vector<torch::jit::IValue> &input)
  {
    torch::NoGradGuard no_grad;
    torch::Tensor output = module->forward(input).toTensor();
    // std::cout << "Device type: " << (output.device().type()) << std::endl;
    // std::cout << "Required grad: " <<  (output.requires_grad()) << std::endl;
    return output;
  }

  void InferenceTorch::Publish(torch::Tensor &out)
  {
    prediction_msg.prediction.clear();
    torch::Device device(torch::kCPU);
    torch::Tensor new_out = out.to(device);

    int grid_size = 128;
    auto output_values = new_out.accessor<float, 5>();
    int probability;

    for (unsigned int t = 0; t < 20; t++)
    {
      occupancy_msg.data.clear();
      occupancy_msg.info.resolution = 1. / 3.;
      occupancy_msg.info.width = grid_size;
      occupancy_msg.info.height = grid_size;
      for (unsigned int i = 0; i < grid_size; i++)
      {
        for (unsigned int j = 0; j < grid_size; j++)
        {
          // Modify this accordingly
          probability = (int)(100 * (0.5 * output_values[0][t][0][i][j] + 0.5 * (1.0 - output_values[0][t][1][i][j])));
          occupancy_msg.data.push_back(probability);
        }
      }
      prediction_msg.prediction.push_back(occupancy_msg);
    }
    pred_all_pub.publish(prediction_msg);
    pred_pub.publish(occupancy_msg);
  }

  void InferenceTorch::createTensorFromFrames(std::vector<torch::jit::IValue> &input)
  {

    bool past = true;
    torch::Tensor input_data = torch::empty({1, 20, 2, 128, 128});
    auto input_map = input_data.accessor<float, 5>();
    lidar_msgs::masses data;
    for (unsigned int t = 0; t < 20; t++)
    {
      if (t < 5)
      {
        data = history_m[t];
      }
      else
      {
        past = false;
      }
      int count = 0;

      for (unsigned int x = 0; x < 128; x++)
      {
        for (unsigned int y = 0; y < 128; y++)
        {
          if (past == true)
          {
            input_map[0][t][0][x][y] = (float)data.occ[count];
            input_map[0][t][1][x][y] = (float)data.free[count];
          }
          count++;
        }
      }
    }
    input.clear();
    input.push_back(input_data.to(torch::kCUDA));
  }

  void InferenceTorch::masses_callback(const lidar_msgs::masses::ConstPtr &masses_data)
  {
    time++;
    nav_msgs::OccupancyGrid zeross;
    // zeross.data = occupancy.data;
    if (time <= 5)
    {
      history_m.push_back(*masses_data);
    }
    else
    {
      data_acquired = true;
      for (unsigned int i = 0; i <= 3; i++)
      {
        history_m[i] = history_m[i + 1];
      }
      history_m[4] = (*masses_data);
    }
  }
}
