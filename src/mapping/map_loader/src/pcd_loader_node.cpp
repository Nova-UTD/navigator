#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
// #include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ros/time.h>
// #include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;


/*

PSEUDOCODE
Every n seconds, 
*/

class PcdLoader : public rclcpp::Node
{
  public:
    PcdLoader()
    : Node("pcd_loader_node")
    {
        this->declare_parameter<std::string>("pcd_path", "/home/main/voltron/grid_divider/");
        this->declare_parameter<double>("min_x", -233.0);
        this->declare_parameter<double>("min_y", -747.0);
        this->declare_parameter<double>("cell_size", 25.0); // Side length of map cell, meters
        m_update_interval = this->declare_parameter<double>("update_interval", 3.0); // In seconds
        m_pcd_viz_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map/pcd", 10);
        m_gps_odom_sub = this->create_subscription<Odometry>("/gps/odom", 10,
          [this](Odometry::SharedPtr msg) { gps_cb(msg); }
        );
        // m_pub_timer = this->create_wall_timer(std::chrono::duration<double>(m_update_interval), std::bind(&PcdLoader::publish_cells, this));
        // visualizeLanelets(_map);
    }

  private:
    void gps_cb(Odometry::SharedPtr msg) {
/*
      PSEUDOCODE
      We want to publish five cells in a plus shape: The cell that the car is in and the 4 adjacent ones.
      Get the current base_link location using a tf lookup.
      Convert this location to a cell row and column: 
      - row = y/cell_size (integer division rounds down, which is what we want)
      - column = x/cell_size
      Load center PCD: {row}_{column}.pcd
      Convert to PointCloud2 and publish
      */
      double min_x, min_y;
      this->get_parameter("cell_size", m_cell_size);
      this->get_parameter<double>("min_x", min_x);
      this->get_parameter<double>("min_y", min_y);
      double current_x = msg->pose.pose.position.x;
      double current_y = msg->pose.pose.position.y;
      int current_cell_x = (current_x-min_x)/m_cell_size;
      int current_cell_y = (current_y-min_y)/m_cell_size;
      RCLCPP_INFO_STREAM(this->get_logger(), "Current cell: "<<current_cell_x<<" "<<current_cell_y);

      publishCell(current_cell_x,   current_cell_y);
      publishCell(current_cell_x+1, current_cell_y); // Publish eastern neighbor
      publishCell(current_cell_x-1, current_cell_y); // Western
      publishCell(current_cell_x, current_cell_y+1); // Northern
      publishCell(current_cell_x, current_cell_y-1); // Souuthern

    }

    void publishCell(int cellx, int celly) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      PointCloud2 output_cloud;
      this->get_parameter("pcd_path", m_pcd_path);
      std::string map_file_path = m_pcd_path+std::to_string(cellx)+"_"+std::to_string(celly)+".pcd";
      

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_file_path, *cloud) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't read map "<<m_pcd_path);
        rclcpp::shutdown();
      } else {
         RCLCPP_INFO(this->get_logger(), "PCD map has been loaded");
      }

      pcl::toROSMsg(*cloud, output_cloud);
      RCLCPP_INFO(this->get_logger(), std::to_string(output_cloud.width));
      RCLCPP_INFO(this->get_logger(), "PCD map has been converted");
      output_cloud.header.frame_id = "map";
      output_cloud.header.stamp = this->now();
      m_pcd_viz_publisher->publish(output_cloud);
    }


    std::string m_pcd_path;
    PointCloud2 m_map_msg;
    double m_update_interval;
    double m_cell_size;
    rclcpp::Subscription<Odometry>::SharedPtr m_gps_odom_sub;
    rclcpp::TimerBase::SharedPtr m_pub_timer;
    rclcpp::Publisher<PointCloud2>::SharedPtr m_pcd_viz_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdLoader>());
  rclcpp::shutdown();
  return 0;
}