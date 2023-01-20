#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <lidar_msgs/point_types.h>
#include <lidar_msgs/vehicle_data.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/convert.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

namespace lidar_pkg
{
  class CloudFilter
  {
    public:
      CloudFilter(){};
      CloudFilter(int min_intensity, double min_distance, double max_distance, double min_height, double max_height, double max_local_height){
        // this->device_id = device_id;
        // this->beam_id = beam_id;
        this->min_intensity = min_intensity;
        this->min_distance = min_distance;
        this->max_distance = max_distance;
        this->min_height = min_height;
        this->max_height = max_height;
        this->max_local_height = max_local_height;
      }

    void filter(sensor_msgs::PointCloud2 &cloud);
    void setVehicle(lidar_msgs::VehicleData vehicle);

    private:
      // int device_id;
      // int beam_id;
      int min_intensity;
      double min_distance;
      double max_distance;
      double min_height;
      double max_height;
      double max_local_height;
      pcl::PointCloud<pcl::PointXYZ> filt_cloud;

      lidar_msgs::VehicleData vehicle;
      double E_m = 0.0;
      double N_m = 0.0;

      bool isDeviceAndBeam(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isAboveIntensity(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isAboveDistance(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isBelowHeight(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isAboveHeight(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isLocallyBelowHeight(pcl::PointCloud<pcl::PointXYZ> pt);
      bool isBelowMaxDistance(pcl::PointCloud<pcl::PointXYZ> pt);
  };

  class AggregatePoints : public nodelet::Nodelet
  {
    public:
      AggregatePoints(){}
    private:
      ros::Publisher pub;
      pcl::PointCloud<pcl::PointXYZ> agg_cloud;
      pcl::PointCloud<pcl::PointXYZ> yellow_cloud;
      pcl::PointCloud<pcl::PointXYZ> red_cloud;
      pcl::PointCloud<pcl::PointXYZ> blue_cloud;
      pcl::PointCloud<pcl::PointXYZ> green_cloud;
      pcl::PointCloud<pcl::PointXYZ> velo_cloud;
      lidar_msgs::VehicleData vehicle_msg;
      CloudFilter filter;
      tf::TransformListener tfListener;

      ros::Subscriber yellow_sub;
      ros::Subscriber red_sub;
      ros::Subscriber blue_sub;
      ros::Subscriber green_sub;
      ros::Subscriber velo_sub;
      ros::Subscriber veh_sub;
      ros::Timer timer;
      tf::TransformListener listener;

      virtual void onInit();
      void aggregate_cloud();
      void yellow_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
      void red_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
      void blue_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
      void green_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
      void velo_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
      void vehicleCallback(const lidar_msgs::vehicle_state& vehicleState);
      void timerCallback(const ros::TimerEvent&);
    };
}
