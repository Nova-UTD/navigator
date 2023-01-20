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

typedef int DST_mass[128][128][3];

namespace lidar_pkg {
  class InferenceTF : public nodelet::Nodelet
  {
  public:
    InferenceTF(){};
  private:
    ros::Publisher pred_pub;
    ros::Publisher pred_all_pub;
    ros::Subscriber occupancy_grid_sub;
    ros::Subscriber masses_sub;
    ros::Timer timer;
    std::vector<std::vector<int>> history;
    std::vector<nav_msgs::OccupancyGrid> history_n;
    std::vector<lidar_msgs::masses> history_m;
    std::unique_ptr<tensorflow::Session> session;
    std::string inputLayer;
    std::string outputLayer;
    nav_msgs::OccupancyGrid occupancy_msg;
    lidar_msgs::masses masses_msg;
    lidar_msgs::prediction prediction_msg;
    int time=0;
    bool data_acquired = false;
    virtual void onInit();
    void timerCallback(const ros::TimerEvent&);
    void createTensorFromFrames(tensorflow::Tensor& input);
    void occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy);
    void createPredictionAndPublish(std::vector<tensorflow::Tensor>& output);
    void masses_callback(const lidar_msgs::masses::ConstPtr& masses_data);
    void publishPrediction();
    void saveFrame(lidar_msgs::masses& data, int t);
    void saveTensor(std::vector<tensorflow::Tensor>& output);
  };
}
