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

typedef int DST_mass[128][128][3];

namespace lidar_pkg {
  class InferenceTorch : public nodelet::Nodelet
  {
  public:
    InferenceTorch(){};
  private:
    ros::Publisher pred_pub;
    ros::Publisher pred_all_pub;
    ros::Subscriber occupancy_grid_sub;
    ros::Subscriber masses_sub;
    ros::Timer timer;
    std::vector<std::vector<int>> history;
    std::vector<nav_msgs::OccupancyGrid> history_n;
    std::vector<lidar_msgs::masses> history_m;
    nav_msgs::OccupancyGrid occupancy_msg;
    lidar_msgs::masses masses_msg;
    lidar_msgs::prediction prediction_msg;
    int time=0;
    bool data_acquired = false;

    //torch::jit::script::Module module;
    std::shared_ptr<torch::jit::script::Module> module;
    //torch::Device device;

    virtual void onInit();
    void timerCallback(const ros::TimerEvent&);
    void createTensorFromFrames(std::vector<torch::jit::IValue>& input);
    void occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy);
    // void createPredictionAndPublish(std::vector<torch::jit::IValue>& input);
    void Publish(torch::Tensor& new_out);
    torch::Tensor Infer(const std::vector<torch::jit::IValue>& input);
    void masses_callback(const lidar_msgs::masses::ConstPtr& masses_data);
    void publishPrediction();
  };
}
