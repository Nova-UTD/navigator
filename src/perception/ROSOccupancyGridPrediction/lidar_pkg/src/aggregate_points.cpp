#include <pluginlib/class_list_macros.h>
#include "aggregate_points.h"

PLUGINLIB_EXPORT_CLASS(lidar_pkg::AggregatePoints, nodelet::Nodelet)

namespace lidar_pkg
{
  void AggregatePoints::onInit()
  {
    ROS_INFO("Initialized AggregatePoints Nodelet");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    pub = private_nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("agg_points",1); //pcl::PointCloud<pcl::PointXYZ> sensor_msgs::PointCloud2
    //filter.setVehicle(vehicle_msg); //???????????????????

    yellow_sub = private_nh.subscribe("velo_yellow_points", 1, &AggregatePoints::yellow_cb, this);
    red_sub = private_nh.subscribe("velo_red_points", 1, &AggregatePoints::red_cb, this);
    blue_sub = private_nh.subscribe("velo_blue_points", 1, &AggregatePoints::blue_cb, this);
    green_sub = private_nh.subscribe("velo_green_points", 1, &AggregatePoints::green_cb, this);
    timer = private_nh.createTimer(ros::Duration(0.1),&AggregatePoints::timerCallback, this);
  }

  void AggregatePoints::timerCallback(const ros::TimerEvent&)
  {
    pub.publish(agg_cloud);
  }

  void AggregatePoints::aggregate_cloud()
  {
    agg_cloud = yellow_cloud;
    this->agg_cloud += this->red_cloud;
    this->agg_cloud += this->green_cloud;
    this->agg_cloud += this->blue_cloud;
  }

  void AggregatePoints::yellow_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcMsg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl_ros::transformPointCloud("body", ros::Time::now(), *temp_cloud, "map",yellow_cloud,tfListener);
    aggregate_cloud();
  }

  void AggregatePoints::red_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcMsg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl_ros::transformPointCloud("body", ros::Time::now(), *temp_cloud, "map",red_cloud,tfListener);
    aggregate_cloud();
  }

  void AggregatePoints::blue_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcMsg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl_ros::transformPointCloud("body", ros::Time::now(), *temp_cloud, "map",blue_cloud,tfListener);
    aggregate_cloud();
  }

  void AggregatePoints::green_cb(const sensor_msgs::PointCloud2::ConstPtr &pcMsg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcMsg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl_ros::transformPointCloud("body", ros::Time::now(), *temp_cloud, "map",green_cloud,tfListener);
    aggregate_cloud();
  }

  //  void CloudFilter::filter(sensor_msgs::PointCloud2 &cloud)
  //  {
  //    ROS_INFO("Filtering...");
  //    this->filt_cloud.header = cloud.header;
  //    this->filt_cloud.clear();
  //
  //    bool include;
  //    for(DDLPointCloud::iterator it = cloud.begin(); it != cloud.end(); it++){
  //      bool include = true;
  //      //if(this->device_id >= 0) include = include && isDeviceAndBeam(*it);
  //      //if(this->min_intensity > 0) include = include && isAboveIntensity(*it);
  //      //if(this->max_height > 0) include = include && isBelowHeight(*it);
  //      // include = include && isAboveHeight(*it);
  //      // include = include && isLocallyBelowHeight(*it);
  //      // include = include && isBelowMaxDistance(*it);
  //      if(include) this->filt_cloud.push_back(*it);
  //    }
  //   cloud = this->filt_cloud;
  // }

   // bool CloudFilter::isDeviceAndBeam(sensor_msgs::PointCloud2 pt){
   //   if(pt.device_id == this->device_id){
   //     if(this->beam_id < 0 || pt.beam_id == this->beam_id){
   //       return true;
   //     }
   //   }
   //   return false;
   // }

   // bool CloudFilter::isAboveIntensity(sensor_msgs::PointCloud2 pt){
   //   if(pt.intensity > this->min_intensity){
   //     return true;
   //   }
   //   return false;
   // }

   // bool CloudFilter::isAboveDistance(sensor_msgs::PointCloud2 pt){
   //   double x_dist = pt.x - E_m;
   //   double y_dist = pt.y - N_m;
   //   if(x_dist*x_dist + y_dist*y_dist > this->min_distance*this->min_distance){
   //     return true;
   //   }
   //   return false;
   // }
   //
   // bool CloudFilter::isBelowHeight(sensor_msgs::PointCloud2 pt){
   //   if(pt.z < this->max_height) return true;
   //   return false;
   // }
   //
   // bool CloudFilter::isAboveHeight(sensor_msgs::PointCloud2 pt){
   //   if(pt.z > this->min_height) return true;
   //   return false;
   // }
   //
   // bool CloudFilter::isLocallyBelowHeight(sensor_msgs::PointCloud2 pt){
   //   if(!isAboveDistance(pt) && pt.z >= this->max_local_height){
   //     return false;
   //  sensor_msgs::PointCloud22
   //   return true;
   // }
   //
   // bool CloudFilter::isBelowMaxDistance(sensor_msgs::PointCloud2 pt){
   //   double x_dist = pt.x - E_m;
   //   double y_dist = pt.y - N_m;
   //   if(x_dist*x_dist + y_dist*y_dist < this->max_distance*this->max_distance){
   //     return true;
   //   }
   //   return false;
   // }

   // void CloudFilter::setVehicle(lidar_msgs::VehicleData vehicle){
   //   this->vehicle = vehicle;
   //   double psi_rad;
   //   this->vehicle.getPose(this->E_m, this->N_m, psi_rad);
   //   // ROS_INFO("GETTING POSE in CloudFilter::setVehicle");
   //   // std::cout << "E_m and N_m: " << E_m << " " << N_m << std::endl;
   // }
}
