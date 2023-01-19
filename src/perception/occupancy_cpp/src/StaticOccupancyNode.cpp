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
  cloud = createOccupancyGrid(cloud);

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

      int indx_nl = find_nearest(grid_size, xstart, x_new_low, x_new_high, res);
      int indx_nh = find_nearest(grid_size, xend, x_new_low, x_new_high, res);
      int indy_nl = find_nearest(grid_size, ystart, y_new_low, y_new_high, res);
      int indy_nh = find_nearest(grid_size, yend, y_new_low, y_new_high, res);

      int indx_ol = find_nearest(grid_size, xstart, x_old_low, x_old_high, res);
      int indx_oh = find_nearest(grid_size, xend, x_old_low, x_old_high, res);
      int indy_ol = find_nearest(grid_size, ystart, y_old_low, y_old_high, res);
      int indy_oh = find_nearest(grid_size, yend, y_old_low, y_old_high, res);

      for (unsigned int i = 0; i < indx_oh - indx_ol + 1; i++)
      {
        for (unsigned int j = 0; j < indy_oh - indy_ol + 1; j++)
        {
          prev_free[indx_nl + i][indy_nl + j] = up_free[indx_ol + i][indy_ol + j];
          prev_occ[indx_nl + i][indy_nl + j] = up_occ[indx_ol + i][indy_ol + j];
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
pcl::PointCloud<pcl::PointXYZI> StaticOccupancyNode::createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> cloud)
{
  // The grid is created in three stages.

  // Initialize DST grid
  pcl::PointCloud<pcl::PointXYZI> grid;

  // 1. Add occupied cells to the DST grid
  add_points_to_the_DST(cloud);

  // 2. Identify free space in the DST grid.
  add_free_spaces_to_the_DST();

  // 3. Add an ego vehicle mask to the grid.
  addEgoMask();

  return grid;
}

/**
 * @brief Fills and adds points to the DST grid
 * It projects the pcl points onto the 2D occupancy grid.
 *
 * @param grid
 */
void StaticOccupancyNode::add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  std::printf("Adding %i points to the DST.\n", cloud.points.size());
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    int x = (int)(cloud.points[i].x / res);
    int y = (int)(cloud.points[i].y / res);
    double z = cloud.points[i].z;

    // Ignores points above a certain height?
    if (z * (-1) > 0.5)
    {
      std::printf("Point was above max height, skipping.\n");
      continue;
    }

    if (x < -1 * SIZE && y < -1 * SIZE && x >= SIZE && y >= SIZE)
    {
      std::printf("Point was outside grid boundaries, skipping.\n");
      continue;
    }

    
    int angle;

    double pi = 3.14159265;

    // Angles vector contians which angles from 0 deg to 360 deg have been represented.
    // It is used to identify free spaces for angles not covered by PC or out of range points.
    if (cloud.points[i].y > 0 && cloud.points[i].x < 0)
    {
      angle = 180 - (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / pi);
    }
    else if (cloud.points[i].y < 0 && cloud.points[i].x < 0)
    {
      angle = 180 + (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / pi);
    }
    else if (cloud.points[i].y < 0 && cloud.points[i].x > 0)
    {
      angle = 360 - (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / pi);
    }
    else
    {
      angle = (int)(atan(std::abs(cloud.points[i].y) / std::abs(cloud.points[i].x)) * 180.0 / pi);
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
  }
}

/**
 * Adds unoccupied spaces to DST using ray tracing
 */
void StaticOccupancyNode::add_free_spaces_to_the_DST()
{
  double i = 0.0;
  float ang = 0.0f;

  // fills free spaces, not efficient?
  for (unsigned int i = 0; i < 3600; i++)
  {
    ang = (i * 0.1f);

    if (angles[(int)(ang)] == false)
    {
      int x, y;
      if (ang > 0.0f && ang <= 45.0f)
      {
        x = 64;
        y = (int)(tan(ang * PI / 180.0f) * x);
      }
      else if (ang > 45.0f && ang < 90.0f)
      {
        y = 64;
        x = (int)(y / tan(ang * PI / 180.0f));
      }
      else if (ang > 90.0f && ang <= 135.0f)
      {
        y = 64;
        x = (int)(y / tan((ang - 180.0f) * PI / 180.0f));
      }
      else if (ang > 135.0f && ang < 180.0f)
      {
        x = -64;
        y = (int)(tan((ang - 180.0) * PI / 180.0f) * x);
      }
      else if (ang > 180.0f && ang <= 225.0f)
      {
        x = -64;
        y = (int)(tan((ang - 180.0f) * PI / 180.0f) * x);
      }
      else if (ang > 225.0f && ang < 270.0f)
      {
        y = -64;
        x = (int)(y / tan((ang - 180.0f) * PI / 180.0f));
      }
      else if (ang > 270.0f && ang <= 315.0f)
      {
        y = -64;
        x = (int)(y / tan((ang - 360.0f) * PI / 180.0f));
      }
      else if (ang > 315.0f && ang < 360.0f)
      {
        x = 64;
        y = (int)(tan((ang - 360.0f) * PI / 180.0f) * x);
      }
      else if (ang == 0.0f || ang == 360.0f)
      {
        ray_tracing_horizontal(64);
        continue;
      }
      else if (ang == 90.0f)
      {
        ray_tracing_vertical(64);
        continue;
      }
      else if (ang == 180.0f)
      {
        ray_tracing_horizontal_n(-64);
        continue;
      }
      else if (ang == 270.0f)
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
    angles[(int)(ang)] = false;
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
      meas_occ[j][i] = 1.0;
      meas_free[j][i] = 0.0;
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
      up_occ_pred[i][j] = std::min(alpha * prev_occ[i][j], 1.0 - prev_free[i][j]);
      up_free_pred[i][j] = std::min(alpha * prev_free[i][j], 1.0 - prev_occ[i][j]);
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
      double unknown_pred = 1.0 - up_free_pred[i][j] - up_occ_pred[i][j];
      double meas_cell_unknown = 1.0 - meas_free[i][j] - meas_occ[i][j];
      double k_value = up_free_pred[i][j] * meas_occ[i][j] + up_occ_pred[i][j] * meas_free[i][j];
      up_occ[i][j] = (up_occ_pred[i][j] * meas_cell_unknown + unknown_pred * meas_occ[i][j] + up_occ_pred[i][j] * meas_occ[i][j]) / (1.0 - k_value);
      up_free[i][j] = (up_free_pred[i][j] * meas_cell_unknown + unknown_pred * meas_free[i][j] + up_free_pred[i][j] * meas_free[i][j]) / (1.0 - k_value);
    }
  }
}

/**
 * @brief
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
      double probability = (0.5 * up_occ[i][j] + 0.5 * (1.0 - up_free[i][j]));
      row.push_back(probability);
    }
    cell_probabilities.push_back(row);
  }

  return cell_probabilities;
}

int StaticOccupancyNode::find_nearest(int n, double v, double v0, double vn, double res)
{
  int idx = std::floor(n * (v - v0 + res / 2.) / (vn - v0 + res));
  return idx;
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
    if (meas_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass)
    {
      break;
    }

    meas_free[flip_x * x + 64][flip_y * y + 64] = meas_mass;

    slope_error += slope;
    if (slope_error >= 0)
    {
      y += 1;
      slope_error -= 2 * (x2 - x1);
    }
  }

  if (inclusive == false)
  {
    meas_occ[flip_x * x2 + 64][flip_y * y2 + 64] = meas_mass;
    meas_free[flip_x * x2 + 64][flip_y * y2 + 64] = 0.0;
  }
}

void StaticOccupancyNode::ray_tracing_approximation_x_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive)
{
  int slope = 2 * (x2 - x1);
  int slope_error = slope - (y2 - y1);
  int x_sample, y_sample;
  for (int x = x1, y = y1; y < y2; y++)
  {
    if (meas_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass)
    {
      break;
    }

    meas_free[flip_x * x + 64][flip_y * y + 64] = meas_mass;

    slope_error += slope;
    if (slope_error >= 0)
    {
      x += 1;
      slope_error -= 2 * (y2 - y1);
    }
  }

  if (inclusive == false)
  {
    meas_occ[flip_x * x2 + 64][flip_y * y2 + 64] = meas_mass;
    meas_free[flip_x * x2 + 64][flip_y * y2 + 64] = 0.0;
  }
}

void StaticOccupancyNode::ray_tracing_horizontal(int x2)
{
  int x1 = 0;
  int y1 = 0;
  x2 = x2 - 1;

  for (int x = x1; x <= x2; x++)
  {
    if (meas_occ[x + 64][64] == meas_mass)
    {
      break;
    }

    meas_free[x + 64][64] = meas_mass;
  }

  meas_free[x2 + 64][64] = 0.0;
}

void StaticOccupancyNode::ray_tracing_horizontal_n(int x1)
{
  int x2 = 0;
  int y2 = 0;
  x1 = x1 + 1;

  for (int x = x1; x <= x2; x++)
  {
    if (meas_occ[x + 64][64] == meas_mass)
    {
      break;
    }

    meas_free[x + 64][64] = meas_mass;
  }

  meas_free[x2 + 64][64] = 0.0;
}

void StaticOccupancyNode::ray_tracing_vertical(int y2)
{
  int x1 = 0;
  int y1 = 0;
  y2 = y2 - 1;

  for (int y = y1; y <= y2; y++)
  {
    if (meas_occ[64][y + 64] == meas_mass)
    {
      break;
    }
    meas_free[64][y + 64] = meas_mass;
  }
}

void StaticOccupancyNode::ray_tracing_vertical_n(int y1)
{
  int x1 = 0;
  int y2 = 0;
  y1 = y1 + 1;

  for (int y = y1; y <= y2; y++)
  {
    if (meas_occ[64][y + 64] == meas_mass)
    {
      break;
    }
    meas_free[64][y + 64] = meas_mass;
  }

  meas_free[64][y2 + 64] = 0.0;
}

void StaticOccupancyNode::clear()
{
  for (unsigned int i = 0; i < grid_size; i++)
  {
    for (unsigned int j = 0; j < grid_size; j++)
    {
      meas_occ[i][j] = 0.0;
      meas_free[i][j] = 0.0;
    }
  }
}
