/*
 * Package:   occupancy_cpp
 * Filename:  StaticOccupancyNode.cpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "occupancy_cpp/StaticOccupancyNode.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

StaticOccupancyNode::StaticOccupancyNode() : Node("ground_segmentation_node")
{
  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  pcd_sub = this->create_subscription<PointCloud2>(
      "/lidar/fused", 10,
      std::bind(&StaticOccupancyNode::pointCloudCb, this, std::placeholders::_1));

  occupancy_grid_pub = this->create_publisher<OccupancyGrid>("/grid/occupancy/current", 10);
}

/**
 * @brief Each time that raw LiDAR is received, remove the ground points and publish the filtered version.
 *
 * @param raw_cloud The unfiltered LiDAR point cloud
 */
void StaticOccupancyNode::pointCloudCb(PointCloud2::SharedPtr msg)
{
/*
    PC processing.
    */

    double centerpoint_x = 64;
    double centerpoint_y = 64;
    double xstart = -1;
    double ystart = -1;
    double xend = -1;
    double yend = -1;

    // Converts the PCL ros message using pcl_conversions.
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 1. Convert new measurement into a DST grid.
    createOccupancyGrid(cloud);

    x_new_low = change_x - 64 * res;
    x_new_high = change_x + 64 * res;
    y_new_low = change_y - 64 * res;
    y_new_high = change_y + 64 * res;

    if(initialization_phase == false) {
      if ((x_new_low >= x_old_low) && (x_old_high >= x_new_low)) {
        xstart = x_new_low;
        xend = x_old_high;
      }

      if ((y_new_low >= y_old_low) && (y_old_high >= y_new_low)) {
        ystart = y_new_low;
        yend = y_old_high;
      }

      if ((x_new_low < x_old_low) && (x_new_high >= x_old_low)) {
        xstart = x_old_low;
        xend = x_new_high;
      }

      if ((y_new_low < y_old_low) && (y_new_high >= y_old_low)) {
        ystart = y_old_low;
        yend = y_new_high;
      }

      if ((xstart != -1) && (ystart != -1)) {

        int indx_nl = find_nearest(grid_size, xstart, x_new_low, x_new_high, res);
        int indx_nh = find_nearest(grid_size, xend, x_new_low, x_new_high, res);
        int indy_nl = find_nearest(grid_size, ystart, y_new_low, y_new_high, res);
        int indy_nh = find_nearest(grid_size, yend, y_new_low, y_new_high, res);

        int indx_ol = find_nearest(grid_size, xstart, x_old_low, x_old_high, res);
        int indx_oh = find_nearest(grid_size, xend, x_old_low, x_old_high, res);
        int indy_ol = find_nearest(grid_size, ystart, y_old_low, y_old_high, res);
        int indy_oh = find_nearest(grid_size, yend, y_old_low, y_old_high, res);

        for (unsigned int i=0; i < indx_oh - indx_ol + 1; i++) {
          for (unsigned int j=0; j < indy_oh - indy_ol + 1; j++) {
            prev_free[indx_nl + i][indy_nl + j] = up_free[indx_ol + i][indy_ol + j];
            prev_occ[indx_nl + i][indy_nl + j] = up_occ[indx_ol + i][indy_ol + j];
          }
        }
      }
    }
    mass_update();
    get_mass();
    plotting();
    clear();

    initialization_phase = false;
    x_old_low = x_new_low;
    x_old_high = x_new_high;
    y_old_low = y_new_low;
    y_old_high = y_new_high;
}

/**
 * @brief Returns a grid using Dempster-Shafer Theory (DST)
 * 
 * @param cloud 
 * @return pcl::PointCloud<pcl::PointXYZI> 
 */
pcl::PointCloud<pcl::PointXYZI> StaticOccupancyNode::createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> cloud)
{
  // The grid is created in three stages.
  // 1. Add occupied cells to the DST grid

  // 2. Identify free space in the DST grid.

  // 3. Add an ego vehicle mask to the grid.

}