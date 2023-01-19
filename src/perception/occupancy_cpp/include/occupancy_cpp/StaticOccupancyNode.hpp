/*
 * Package:   occupancy_cpp
 * Filename:  StaticOccupancyNode.hpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once


// Message definitions
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nova_msgs/msg/masses.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"

#define PI 3.14159

using namespace std::chrono_literals;

using nav_msgs::msg::OccupancyGrid;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;
// using nova_msgs::msg::Masses;

namespace navigator
{
  namespace perception
  {

    class StaticOccupancyNode : public rclcpp::Node
    {
    public:
      StaticOccupancyNode();
      virtual ~StaticOccupancyNode();

    private:

      // Publishers
      rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub;
      // rclcpp::Publisher<Masses>::SharedPtr masses_pub;

      // Subscribers
      rclcpp::Subscription<Clock>::SharedPtr clock_sub;
      rclcpp::Subscription<PointCloud2>::SharedPtr pcd_sub;

      // Callbacks

      // Timers
      // rclcpp::TimerBase::SharedPtr map_marker_timer;

      Clock clock;

      pcl::PointCloud<pcl::PointXYZI> createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> cloud);

      bool initialization_phase = true;
      double vehicle_x;
      double vehicle_y;
      double prev_vehicle_x;
      double prev_vehicle_y;
      double change_x;
      double change_y;
      double x_new_low;
      double x_new_high;
      double y_new_low;
      double y_new_high;
      double x_old_low;
      double x_old_high;
      double y_old_low;
      double y_old_high;

      // There are only two events: 0 = Occupied and 1 = Free.
      const static int event_num = 2;

      // Grid size.
      const static int grid_size = 128;

      const int SIZE = 64;

      // Resolution.
      constexpr static double res = 1. / 3.;

      // Measurement mass.
      constexpr static double meas_mass = 0.95;

      // Occupancy measurement.
      constexpr static double alpha = 0.9;

      // Place holders vehicle positions.
      constexpr static double vehicle_pos_x = 0;
      constexpr static double vehicle_pos_y = 0;
      constexpr static double prev_vehicle_pos_x = 0;
      constexpr static double prev_vehicle_pos_y = 0;

      //rclcpp::TimerBase::SharedPtr timer;
      OccupancyGrid occupancy_msg;
      // Masses masses_msg;

      // Array with the DST data.
      double meas_grids[event_num][grid_size][grid_size];

      // "Freeness" measurement.
      double meas_free[grid_size][grid_size] = {{0}};

      // Occupancy measurement.
      double meas_occ[grid_size][grid_size] = {{0}};
      double prev_free[grid_size][grid_size] = {{0}};
      double prev_occ[grid_size][grid_size] = {{0}};
      double up_free_pred[grid_size][grid_size];
      double up_occ_pred[grid_size][grid_size];
      double up_free[grid_size][grid_size];
      double up_occ[grid_size][grid_size];
      double prob_O[grid_size][grid_size];
      double prob_O_plot[grid_size][grid_size];
      bool angles[360];
      bool first;

      //void timer_cb(const ros::TimerEvent &);

      void transform_listener();
      void pointCloudCb(const PointCloud2::SharedPtr msg);
      void create_DST_grid(pcl::PointCloud<pcl::PointXYZI> &cloud);
      void ray_tracing_approximation_x_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive);
      void ray_tracing_approximation_y_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive);
      int find_nearest(int n, double v, double v0, double vn, double res);
      void mass_update();
      void update_of();
      std::vector<std::vector<double>> getGridCellProbabilities();
      void plotting();
      void clear();
      void fill(std::vector<int> flip);
      void ray_tracing_horizontal(int y);
      void ray_tracing_horizontal_n(int y);
      void ray_tracing_vertical(int x);
      void ray_tracing_vertical_n(int x);
      void add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud);
      void add_free_spaces_to_the_DST();
      void add_ego_vehicle_to_the_DST();
    };
  }
}
