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

StaticOccupancyNode::StaticOccupancyNode() : Node("static_occupancy_node")
{
  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  pcd_sub = this->create_subscription<PointCloud2>(
      "/lidar/fused",
      10,
      std::bind(&StaticOccupancyNode::pointCloudCb, this, std::placeholders::_1));

  occupancy_grid_pub = this->create_publisher<OccupancyGrid>("/grid/occupancy/current", 10);
  // masses_pub = this->create_publisher<nova_msgs::msg::Masses>("masses", 10);
}

StaticOccupancyNode::~StaticOccupancyNode()
{
  // nothing here
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
  printf("Up_Occ: %d, Up_Free: %d\n", updated_occ[50][50], updated_free[50][50]);
  printf("Meas_Occ: %d\n", measured_occ[50][50]);
  printf("Prev_Occ: %d, Prev_Free: %d\n", previous_occ[50][50], previous_free[50][50]);
  

  change_x = 0.0; // TODO: Remove or fix the "change" variables.
  change_y = 0.0;

  x_new_low = change_x - 64 * res;
  x_new_high = change_x + 64 * res;
  y_new_low = change_y - 64 * res;
  y_new_high = change_y + 64 * res;

  if (initialization_phase == false)
  {
    if ((x_new_low >= x_old_low) && (x_old_high >= x_new_low))
    {
      xstart = x_new_low;
      xend = x_old_high;
    }

    if ((y_new_low >= y_old_low) && (y_old_high >= y_new_low))
    {
      ystart = y_new_low;
      yend = y_old_high;
    }

    if ((x_new_low < x_old_low) && (x_new_high >= x_old_low))
    {
      xstart = x_old_low;
      xend = x_new_high;
    }

    if ((y_new_low < y_old_low) && (y_new_high >= y_old_low))
    {
      ystart = y_old_low;
      yend = y_new_high;
    }

    if ((xstart != -1) && (ystart != -1))
    {
      
      /**
       * NL = New Low, OH = Old High
       */

      //x
      int index_xNL = find_nearest(grid_size, xstart, x_new_low, x_new_high, res);
      int index_xNH = find_nearest(grid_size, xend, x_new_low, x_new_high, res);
      int index_xOL = find_nearest(grid_size, xstart, x_old_low, x_old_high, res);
      int index_xOH = find_nearest(grid_size, xend, x_old_low, x_old_high, res);
      //y
      int index_yNL = find_nearest(grid_size, ystart, y_new_low, y_new_high, res);
      int index_yNH = find_nearest(grid_size, yend, y_new_low, y_new_high, res);
      int index_yOL = find_nearest(grid_size, ystart, y_old_low, y_old_high, res);
      int index_yOH = find_nearest(grid_size, yend, y_old_low, y_old_high, res);

      for (unsigned int i = 0; i < index_xOH - index_xOL + 1; i++)
      {
        for (unsigned int j = 0; j < index_yOH - index_yOL + 1; j++)
        {
          previous_free[index_xNL + i][index_yNL + j] = updated_free[index_xOL + i][index_yOL + j];
          previous_occ[index_xNL + i][index_yNL + j] = updated_occ[index_xOL + i][index_yOL + j];
        }
      }
    }
  }

  mass_update();
  // get_mass();
  publishOccupancyGrid();
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
void StaticOccupancyNode::createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  // The grid is created in three stages.

  // Initialize DST grid
  // pcl::PointCloud<pcl::PointXYZI> grid;

  // 1. Add occupied cells to the DST grid
  add_points_to_the_DST(cloud);
  // 2. Identify free space in the DST grid.
  add_free_spaces_to_the_DST();

  // 3. Add an ego vehicle mask to the grid.
  addEgoMask();

  //return grid;
}

/**
 * @brief Fills and adds points to the DST grid
 * It projects the pcl points onto the 2D occupancy grid.
 *
 * @param grid
 */
void StaticOccupancyNode::add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  std::printf("Adding %i points to the DST.\n", cloud.size());
  for (size_t i = 0; i < cloud.size(); i++)
  {

    int x = (int)(cloud[i].x / res);
    int y = (int)(cloud[i].y / res);
    double z = cloud.points[i].z;

    // Ignores points above a certain height?
    if (z * (-1) > 0.5)
    {
      std::printf("Point was above max height, skipping.\n");
      continue;
    }
    else if (x < -1 * SIZE && y < -1 * SIZE && x >= SIZE && y >= SIZE)
    {
      std::printf("Point was outside grid boundaries, skipping.\n");
      continue;
    }

    int angle;

    // Angles vector contians which angles from 0 deg to 360 deg have been represented.
    // It is used to identify free spaces for angles not covered by PC or out of range points.
    if (cloud.points[i].y > 0 && cloud.points[i].x < 0)
    {
      angle = 180 - (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / M_PI);
    }
    else if (cloud.points[i].y < 0 && cloud.points[i].x < 0)
    {
      angle = 180 + (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / M_PI);
    }
    else if (cloud.points[i].y < 0 && cloud.points[i].x > 0)
    {
      angle = 360 - (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / M_PI);
    }
    else
    {
      angle = (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / M_PI);
    }

    angles[angle] = true;
    double slope = (double)(y) / (x);

    // ray tracing from origin to point, identifies free space using Bresenhaum's line algo
    if (slope > 0 && slope <= 1 && x > 0)
    {
      ray_tracing_approximation_y_increment(0, 0, x, y, 1, 1, false);
    }
    else if (slope > 1 && x > 0)
    {
      ray_tracing_approximation_x_increment(0, 0, x, y, 1, 1, false);
    }
    else if (slope < 0 && slope >= -1 && x > 0)
    {
      ray_tracing_approximation_y_increment(0, 0, x, (-1) * y, 1, -1, false);
    }
    else if (slope < -1 && x > 0)
    {
      ray_tracing_approximation_x_increment(0, 0, x, (-1) * y, 1, -1, false);
    }
    else if (slope > 1 && x < 0)
    {
      ray_tracing_approximation_x_increment(0, 0, (-1) * x, (-1) * y, -1, -1, false);
    }
    else if (slope > 0 && slope <= 1 && x < 0)
    {
      ray_tracing_approximation_y_increment(0, 0, (-1) * x, (-1) * y, -1, -1, false);
    }
    else if (slope < 0 && slope >= -1 && x < 0)
    {
      ray_tracing_approximation_y_increment(0, 0, (-1) * x, y, -1, 1, false);
    }
    else if (slope < -1 && x < 0)
    {
      ray_tracing_approximation_x_increment(0, 0, (-1) * x, y, -1, 1, false);
    }
    else {
      printf("Did not match a case.\n");
      //printf("X: %f, Y: %f, SLOPE %f\n", x, y, slope);
    }
  }
}

/**
 * Adds unoccupied spaces to DST using ray tracing
 */
void StaticOccupancyNode::add_free_spaces_to_the_DST()
{
  double i = 0.0;
  float angle = 0.0f;

  // fills free spaces, not efficient?
  for (unsigned int i = 0; i < 3600; i++)
  {
    angle = (i * 0.1f);

    if (angles[(int)(angle)] == false)
    {
      int x, y;
      if (angle > 0.0f && angle <= 45.0f)
      {
        x = 64;
        y = (int)(tan(angle * M_PI / 180.0f) * x);
      }
      else if (angle > 45.0f && angle < 90.0f)
      {
        y = 64;
        x = (int)(y / tan(angle * M_PI / 180.0f));
      }
      else if (angle > 90.0f && angle <= 135.0f)
      {
        y = 64;
        x = (int)(y / tan((angle - 180.0f) * M_PI / 180.0f));
      }
      else if (angle > 135.0f && angle < 180.0f)
      {
        x = -64;
        y = (int)(tan((angle - 180.0) * M_PI / 180.0f) * x);
      }
      else if (angle > 180.0f && angle <= 225.0f)
      {
        x = -64;
        y = (int)(tan((angle - 180.0f) * M_PI / 180.0f) * x);
      }
      else if (angle > 225.0f && angle < 270.0f)
      {
        y = -64;
        x = (int)(y / tan((angle - 180.0f) * M_PI / 180.0f));
      }
      else if (angle > 270.0f && angle <= 315.0f)
      {
        y = -64;
        x = (int)(y / tan((angle - 360.0f) * M_PI / 180.0f));
      }
      else if (angle > 315.0f && angle < 360.0f)
      {
        x = 64;
        y = (int)(tan((angle - 360.0f) * M_PI / 180.0f) * x);
      }
      else if (angle == 0.0f || angle == 360.0f)
      {
        ray_tracing_horizontal(64);
        continue;
      }
      else if (angle == 90.0f)
      {
        ray_tracing_vertical(64);
        continue;
      }
      else if (angle == 180.0f)
      {
        ray_tracing_horizontal_n(-64);
        continue;
      }
      else if (angle == 270.0f)
      {
        ray_tracing_vertical_n(-64);
        continue;
      }

      if (x >= -64 && y >= -64 && x <= 64 && y <= 64)
      {
        double slope = (double)(y) / (x);

        if (slope > 0 && slope <= 1 && x > 0)
        {
          ray_tracing_approximation_y_increment(0, 0, x, y, 1, 1, true);
        }
        else if (slope > 1 && x > 0)
        {
          ray_tracing_approximation_x_increment(0, 0, x, y, 1, 1, true);
        }
        else if (slope < 0 && slope >= -1 && x > 0)
        {
          ray_tracing_approximation_y_increment(0, 0, x, (-1) * y, 1, -1, true);
        }
        else if (slope < -1 && x > 0)
        {
          ray_tracing_approximation_x_increment(0, 0, x, (-1) * y, 1, -1, true);
        }
        else if (slope > 1 && x < 0)
        {
          ray_tracing_approximation_x_increment(0, 0, (-1) * x, (-1) * y, -1, -1, true);
        }
        else if (slope > 0 && slope <= 1 && x < 0)
        {
          ray_tracing_approximation_y_increment(0, 0, (-1) * x, (-1) * y, -1, -1, true);
        }
        else if (slope < 0 && slope >= -1 && x < 0)
        {
          ray_tracing_approximation_y_increment(0, 0, (-1) * x, y, -1, 1, true);
        }
        else if (slope < -1 && x < 0)
        {
          ray_tracing_approximation_x_increment(0, 0, (-1) * x, y, -1, 1, true);
        }
      }
    }
    angles[(int)(angle)] = false;
  }
}

/**
 * Adds vehicle shape to occupied/unoccupied zones
 * TODO: Change for Hail-Bopp
 */
void StaticOccupancyNode::addEgoMask()
{
  // Vehicle shape.
  for (unsigned int j = 60; j < 68; j++)
  {
    for (unsigned int i = 62; i < 67; i++)
    {
      measured_occ[j][i] = 1.0;
      measured_free[j][i] = 0.0;
    }
  }
}

//-------------HELPERS----------------------------//
void StaticOccupancyNode::publishOccupancyGrid()
{

  OccupancyGrid msg;

  msg.header.stamp = this->clock.clock;
  msg.header.frame_id = "base_link"; // TODO: Make sure the frame is the correct one.
  msg.info.resolution = res;
  msg.info.width = grid_size;
  msg.info.height = grid_size;
  msg.info.origin.position.z = 0.2;
  msg.info.origin.position.x = -64.0 * (1. / 3.);
  msg.info.origin.position.y = -64.0 * (1. / 3.);
  // masses_msg.width = grid_size;
  // masses_msg.height = grid_size;

  auto probabilites = getGridCellProbabilities();

  for (unsigned int i = 0; i < grid_size; i++)
  {
    for (unsigned int j = 0; j < grid_size; j++)
    {
      msg.data.push_back(probabilites.at(i).at(j));
    }
  }
  occupancy_grid_pub->publish(msg);
  // masses_pub->publish(masses_msg);
}

void StaticOccupancyNode::mass_update()
{
  for (unsigned int i = 0; i < grid_size; i++)
  {
    for (unsigned int j = 0; j < grid_size; j++)
    {
      updated_occP[i][j] = std::min(decay_factor * previous_occ[i][j], 1.0 - previous_free[i][j]);
      updated_freeP[i][j] = std::min(decay_factor * previous_free[i][j], 1.0 - previous_occ[i][j]);
    }
  }
  // Combine measurement nad prediction to form posterior occupied and free masses.
  update_of();
}

void StaticOccupancyNode::update_of()
{
  for (unsigned int i = 0; i < grid_size; i++)
  {
    for (unsigned int j = 0; j < grid_size; j++)
    {
      double unknown_pred = 1.0 - updated_freeP[i][j] - updated_occP[i][j];
      double measured_cell_unknown = 1.0 - measured_free[i][j] - measured_occ[i][j];
      double k_value = updated_freeP[i][j] * measured_occ[i][j] + updated_occP[i][j] * measured_free[i][j];
      updated_occ[i][j] = (updated_occP[i][j] * measured_cell_unknown + unknown_pred * measured_occ[i][j] + updated_occP[i][j] * measured_occ[i][j]) / (1.0 - k_value);
      updated_free[i][j] = (updated_freeP[i][j] * measured_cell_unknown + unknown_pred * measured_free[i][j] + updated_freeP[i][j] * measured_free[i][j]) / (1.0 - k_value);
    }
  }
}

/**
 * @brief Gets the average of the updated occupancy and updated free values and adds to a cell_probabilities
 *
 */
std::vector<std::vector<double>> StaticOccupancyNode::getGridCellProbabilities()
{
  std::vector<std::vector<double>> cell_probabilities;
  for (unsigned int i = 0; i < grid_size; i++)
  {
    std::vector<double> row;
    for (unsigned int j = 0; j < grid_size; j++)
    {
      double probability = (0.5 * updated_occ[i][j] + 0.5 * (1.0 - updated_free[i][j]));
      row.push_back(probability);
    }
    cell_probabilities.push_back(row);
  }

  return cell_probabilities;
}

int StaticOccupancyNode::find_nearest(int n, double v, double v0, double vn, double res)
{
  int index = std::floor(n * (v - v0 + res / 2.) / (vn - v0 + res));
  return index;
}
//------------------------------------------------//

//-------------RAY TRACING HELPERS----------------//
void StaticOccupancyNode::ray_tracing_approximation_y_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive)
{
  int slope = 2 * (y2 - y1);
  int slope_error = slope - (x2 - x1);
  int x_sample, y_sample;
  for (int x = x1, y = y1; x < x2; x++)
  {
    if (measured_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass)
    {
      break;
    }

    measured_free[flip_x * x + 64][flip_y * y + 64] = meas_mass;

    slope_error += slope;
    if (slope_error >= 0)
    {
      y += 1;
      slope_error -= 2 * (x2 - x1);
    }
  }

  if (inclusive == false)
  {
    measured_occ[flip_x * x2 + 64][flip_y * y2 + 64] = meas_mass;
    measured_free[flip_x * x2 + 64][flip_y * y2 + 64] = 0.0;
  }
}

void StaticOccupancyNode::ray_tracing_approximation_x_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive)
{
  int slope = 2 * (x2 - x1);
  int slope_error = slope - (y2 - y1);
  int x_sample, y_sample;
  for (int x = x1, y = y1; y < y2; y++)
  {
    if (measured_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass)
    {
      break;
    }

    measured_free[flip_x * x + 64][flip_y * y + 64] = meas_mass;

    slope_error += slope;
    if (slope_error >= 0)
    {
      x += 1;
      slope_error -= 2 * (y2 - y1);
    }
  }

  if (inclusive == false)
  {
    measured_occ[flip_x * x2 + 64][flip_y * y2 + 64] = meas_mass;
    measured_free[flip_x * x2 + 64][flip_y * y2 + 64] = 0.0;
  }
}

void StaticOccupancyNode::ray_tracing_horizontal(int x2)
{
  int x1 = 0;
  int y1 = 0;
  x2 = x2 - 1;

  for (int x = x1; x <= x2; x++)
  {
    if (measured_occ[x + 64][64] == meas_mass)
    {
      break;
    }

    measured_free[x + 64][64] = meas_mass;
  }

  measured_free[x2 + 64][64] = 0.0;
}

void StaticOccupancyNode::ray_tracing_horizontal_n(int x1)
{
  int x2 = 0;
  int y2 = 0;
  x1 = x1 + 1;

  for (int x = x1; x <= x2; x++)
  {
    if (measured_occ[x + 64][64] == meas_mass)
    {
      break;
    }

    measured_free[x + 64][64] = meas_mass;
  }

  measured_free[x2 + 64][64] = 0.0;
}

void StaticOccupancyNode::ray_tracing_vertical(int y2)
{
  int x1 = 0;
  int y1 = 0;
  y2 = y2 - 1;

  for (int y = y1; y <= y2; y++)
  {
    if (measured_occ[64][y + 64] == meas_mass)
    {
      break;
    }
    measured_free[64][y + 64] = meas_mass;
  }
}

void StaticOccupancyNode::ray_tracing_vertical_n(int y1)
{
  int x1 = 0;
  int y2 = 0;
  y1 = y1 + 1;

  for (int y = y1; y <= y2; y++)
  {
    if (measured_occ[64][y + 64] == meas_mass)
    {
      break;
    }
    measured_free[64][y + 64] = meas_mass;
  }

  measured_free[64][y2 + 64] = 0.0;
}

void StaticOccupancyNode::clear()
{
  for (unsigned int i = 0; i < grid_size; i++)
  {
    for (unsigned int j = 0; j < grid_size; j++)
    {
      measured_occ[i][j] = 0.0;
      measured_free[i][j] = 0.0;
    }
  }
}
