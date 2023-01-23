#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <lidar_msgs/prediction.h>
#include <lidar_msgs/masses.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>

namespace lidar_pkg {

  class Visualize : public nodelet::Nodelet
  {
  public:
    Visualize(){};
  private:
    ros::Publisher visualization_pub;
    ros::Subscriber prediction_sub;
    ros::Timer timer;
    nav_msgs::OccupancyGrid occupancy_msg;
    lidar_msgs::prediction newest_prediction;
    ros::Time lasttime;
    int count = 0;
    bool data_acquired = false;
    virtual void onInit();
    void timerCallback(const ros::TimerEvent&);
    void prediction_callback(const lidar_msgs::prediction::ConstPtr& msg);
    void saveFrame(std::vector<int8_t> data, int t);
  };
}
