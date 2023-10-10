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
#include "navigator_msgs/msg/masses.hpp"
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

using namespace std::chrono_literals;

using nav_msgs::msg::OccupancyGrid;
using navigator_msgs::msg::Masses;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;

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
      rclcpp::Publisher<Masses>::SharedPtr masses_pub;

      // Subscribers
      rclcpp::Subscription<Clock>::SharedPtr clock_sub;
      rclcpp::Subscription<PointCloud2>::SharedPtr pcd_sub;

      // Callbacks

      // Timers
      // rclcpp::TimerBase::SharedPtr map_marker_timer;

      Clock clock;

      void createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> &cloud);

      bool initialization_phase = true;
      float vehicle_x;
      float vehicle_y;
      float prev_vehicle_x;
      float prev_vehicle_y;
      float change_x;
      float change_y;
      float x_new_low;
      float x_new_high;
      float y_new_low;
      float y_new_high;
      float x_old_low;
      float x_old_high;
      float y_old_low;
      float y_old_high;

      // There are only two events: 0 = Occupied and 1 = Free.
      const static int event_num = 2;

      // Grid size.
      const static int GRID_SIZE = 128;

      const int HALF_SIZE = GRID_SIZE / 2;

      // Resolution.
      constexpr static float res = 1. / 3.;

      // Measurement mass.
      constexpr static float meas_mass = 0.95;

      // Occupancy measurement. (ALPHA)
      constexpr static float decay_factor = 0.9;

      // Place holders vehicle positions.
      constexpr static float vehicle_pos_x = 0;
      constexpr static float vehicle_pos_y = 0;
      constexpr static float prev_vehicle_pos_x = 0;
      constexpr static float prev_vehicle_pos_y = 0;

      // Masses masses_msg;

      // Array with the DST data.
      float meas_grids[event_num][GRID_SIZE][GRID_SIZE];

      // "Freeness" measurement.
      float measured_free[GRID_SIZE][GRID_SIZE] = {{0}};

      // Occupancy measurement.
      float measured_occ[GRID_SIZE][GRID_SIZE] = {{0}};
      float previous_free[GRID_SIZE][GRID_SIZE] = {{0}};
      float previous_occ[GRID_SIZE][GRID_SIZE] = {{0}};
      float updated_freeP[GRID_SIZE][GRID_SIZE] = {{0}};
      float updated_occP[GRID_SIZE][GRID_SIZE] = {{0}};
      float updated_free[GRID_SIZE][GRID_SIZE] = {{0}};
      float updated_occ[GRID_SIZE][GRID_SIZE] = {{0}};

      // Masses measurement (probability distribution)
      float probabilities[GRID_SIZE][GRID_SIZE] = {{0}};
      float probabilities_plot[GRID_SIZE][GRID_SIZE] = {{0}};

      bool angles[360];
      bool first;

      // void timer_cb(const ros::TimerEvent &);

      void transform_listener();
      void pointCloudCb(const PointCloud2::SharedPtr msg);
      void create_DST_grid(pcl::PointCloud<pcl::PointXYZI> &cloud);
      void ray_tracing_approximation_x_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive);
      void ray_tracing_approximation_y_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive);
      int find_nearest(int num, float value, float min, float max, float res);
      void update_previous();
      void mass_update();
      void update_of();
      void get_mass();
      void plotting();
      std::vector<std::vector<float>> getGridCellProbabilities();
      void publishOccupancyGrid();
      void clear();
      void fill(std::vector<int> flip);
      void ray_tracing_horizontal(int y);
      void ray_tracing_horizontal_n(int y);
      void ray_tracing_vertical(int x);
      void ray_tracing_vertical_n(int x);
      void add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud);
      void add_free_spaces_to_the_DST();
      void addEgoMask();
    };
  }
}
