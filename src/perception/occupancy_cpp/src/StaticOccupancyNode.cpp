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

/**
 * @brief Constructor for static occupancy node
 * Subscribers: CARLA clock, Ground Segmented Pointcloud
 * Publishers: Static Occupancy Grid, Masses Grid
 */
StaticOccupancyNode::StaticOccupancyNode() : Node("static_occupancy_node")
{
  //------Subscribers-------//
  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  pcd_sub = this->create_subscription<PointCloud2>(
      "/lidar/filtered",
      10,
      std::bind(&StaticOccupancyNode::pointCloudCb, this, std::placeholders::_1));

  //----Publishers-------//
  occupancy_grid_pub = this->create_publisher<OccupancyGrid>("/grid/occupancy/current", 10);
  masses_pub = this->create_publisher<Masses>("/grid/masses", 10);
}

StaticOccupancyNode::~StaticOccupancyNode()
{
  // nothing here
}

/**
 * @brief Each time that raw LiDAR pcl is received:
 * 1. Convert to pcl::PointCloud<pcl::PointXYZI> (x, y, z, intensity), keep in mind this reverses rows and columns?
 * 2. Fills and ray traces static occupancy grid
 *
 * @param msg The LiDAR point cloud previously ground segmented
 */
void StaticOccupancyNode::pointCloudCb(PointCloud2::SharedPtr msg)
{
  // Converts the PCL ros message using pcl_conversions.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // 1. Convert new measurement into a DST grid.
  createOccupancyGrid(cloud);

  // 2. Updates previous grid with updated grid values (important in cases of variable grid size)
  update_previous();

  // 3. Add decayed region (previous grid) to the updated grid
  mass_update();

  // 4. Publish static occupancy grid and mass grid
  publishOccupancyGrid();

  // 5. Clear current measured grid
  clear();
}

/**
 * @brief Returns a grid using Dempster-Shafer Theory (DST)
 * 1. Ray-traces free space towards recorded points (occupied space)
 * 2. Fills the rest of the grid with free space using same ray-tracing algorithms (can combine steps 1 and 2?)
 * 3. Adds occupied space representing the vehicle
 *
 * @param cloud
 * @return pcl::PointCloud<pcl::PointXYZI>
 */
void StaticOccupancyNode::createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  // 1. Ray traces towards occupied spaces
  add_points_to_the_DST(cloud);

  // 2. Ray traces rest of grid to fill with empty space
  add_free_spaces_to_the_DST();

  // 3. Add an ego vehicle mask to the grid.
  // addEgoMask();
}

/**
 * @brief Fills and adds points to the DST grid
 * It projects the pcl points onto the 2D occupancy grid.
 *
 * @param grid
 */
void StaticOccupancyNode::add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  // std::printf("Adding %i points to the DST.\n\n", cloud.size());
  for (size_t i = 0; i < cloud.size(); i++)
  {

    // Dimensions for X & Y [-64 -> 64 (HALF_SiZE)]

    // Record occupancy value for the corresponding point in the pcl, nearest index
    int x = (int)(cloud[i].x / RES);
    int y = (int)(cloud[i].y / RES);

    float z = cloud[i].z;

    // Ignores points above a certain height
    if (z * (-1) > 0.5)
    {
      std::printf("Point was above max height, skipping.\n");
      continue;
    }

    if (x < (-1 * HALF_SIZE) || y < (-1 * HALF_SIZE) || x >= HALF_SIZE || y >= HALF_SIZE)
    {
      // std::printf("Point was outside grid boundaries, skipping.\n");
      continue;
    }

    int angle;

    // Angles vector contians which angles from 0 deg to 360 deg have been represented.
    // It is used to identify free spaces for angles not covered by PC or out of range points.
    if (cloud[i].y > 0 && cloud[i].x < 0)
    {
      angle = 180 - (int)(atan(std::abs(cloud[i].y) / std::abs(cloud[i].x)) * 180.0 / M_PI);
    }
    else if (cloud[i].y < 0 && cloud[i].x < 0)
    {
      angle = 180 + (int)(atan(std::abs(cloud[i].y) / std::abs(cloud[i].x)) * 180.0 / M_PI);
    }
    else if (cloud[i].y < 0 && cloud[i].x > 0)
    {
      angle = 360 - (int)(atan(std::abs(cloud[i].y) / std::abs(cloud[i].x)) * 180.0 / M_PI);
    }
    else
    {
      angle = (int)(atan(std::abs(cloud[i].y) / std::abs(cloud[i].x)) * 180.0 / M_PI);
    }

    angles[angle] = true;

    float slope = (float)(y) / (x);

    // ray tracing from origin to point, identifies free space using Bresenhaum's line algo
    if (slope > 0 && slope <= 1 && x > 0)
    {
      ray_tracing_approximation_y_increment(x, y, 1, 1, false);
    }
    else if (slope > 1 && x > 0)
    {
      ray_tracing_approximation_x_increment(x, y, 1, 1, false);
    }
    else if (slope < 0 && slope >= -1 && x > 0)
    {
      ray_tracing_approximation_y_increment(x, (-1) * y, 1, -1, false);
    }
    else if (slope < -1 && x > 0)
    {
      ray_tracing_approximation_x_increment(x, (-1) * y, 1, -1, false);
    }
    else if (slope > 1 && x < 0)
    {
      ray_tracing_approximation_x_increment((-1) * x, (-1) * y, -1, -1, false);
    }
    else if (slope > 0 && slope <= 1 && x < 0)
    {
      ray_tracing_approximation_y_increment((-1) * x, (-1) * y, -1, -1, false);
    }
    else if (slope < 0 && slope >= -1 && x < 0)
    {
      ray_tracing_approximation_y_increment((-1) * x, y, -1, 1, false);
    }
    else if (slope < -1 && x < 0)
    {
      ray_tracing_approximation_x_increment((-1) * x, y, -1, 1, false);
    }
  }
}

/**
 * Adds unoccupied spaces to DST using ray tracing
 */
void StaticOccupancyNode::add_free_spaces_to_the_DST()
{
  float i = 0.0;
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
        float slope = (float)(y) / (x);

        if (slope > 0 && slope <= 1 && x > 0)
        {
          ray_tracing_approximation_y_increment(x, y, 1, 1, true);
        }
        else if (slope > 1 && x > 0)
        {
          ray_tracing_approximation_x_increment(x, y, 1, 1, true);
        }
        else if (slope < 0 && slope >= -1 && x > 0)
        {
          ray_tracing_approximation_y_increment(x, (-1) * y, 1, -1, true);
        }
        else if (slope < -1 && x > 0)
        {
          ray_tracing_approximation_x_increment(x, (-1) * y, 1, -1, true);
        }
        else if (slope > 1 && x < 0)
        {
          ray_tracing_approximation_x_increment((-1) * x, (-1) * y, -1, -1, true);
        }
        else if (slope > 0 && slope <= 1 && x < 0)
        {
          ray_tracing_approximation_y_increment((-1) * x, (-1) * y, -1, -1, true);
        }
        else if (slope < 0 && slope >= -1 && x < 0)
        {
          ray_tracing_approximation_y_increment((-1) * x, y, -1, 1, true);
        }
        else if (slope < -1 && x < 0)
        {
          ray_tracing_approximation_x_increment((-1) * x, y, -1, 1, true);
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
  for (unsigned int i = 60; i < 68; i++)
  {
    for (unsigned int j = 62; j < 67; j++)
    {
      measured_occ[i][j] = 1.0;
      measured_free[i][j] = 0.0;
    }
  }
}

//-------------HELPERS----------------------------//
void StaticOccupancyNode::publishOccupancyGrid()
{

  //--Occupancy Grid--//
  OccupancyGrid msg;

  msg.header.stamp = this->clock.clock;
  msg.header.frame_id = "base_link"; // TODO: Make sure the frame is the correct one.
  msg.info.resolution = RES;
  msg.info.width = GRID_SIZE;
  msg.info.height = GRID_SIZE;
  msg.info.origin.position.z = 0.2;
  msg.info.origin.position.x = -64.0 * (RES);
  msg.info.origin.position.y = -64.0 * (RES);
  //-----------------//

  //--Masses--//
  Masses masses_msg;
  masses_msg.occ.clear();
  masses_msg.free.clear();
  masses_msg.width = GRID_SIZE;
  masses_msg.height = GRID_SIZE;
  //----------//

  auto probabilities = getGridCellProbabilities();

  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      msg.data.push_back(100 * probabilities.at(j).at(i));
      masses_msg.occ.push_back(updated_occ[i][j]);
      masses_msg.free.push_back(updated_free[i][j]);
    }
  }
  occupancy_grid_pub->publish(msg);
  masses_pub->publish(masses_msg);
}

void StaticOccupancyNode::update_previous()
{
  for (unsigned int i = 0; i < GRID_SIZE; i++)
  {
    for (unsigned int j = 0; j < GRID_SIZE; j++)
    {
      previous_free[i][j] = updated_free[i][j];
      previous_occ[i][j] = updated_occ[i][j];
    }
  }

  //--------CODE FOR VARIABLE INPUT GRID SIZE BELOW----------//

  // float xstart = -1;
  // float ystart = -1;
  // float xend = -1;
  // float yend = -1;

  // change_x = 0.0; // TODO: Remove or fix the "change" variables.
  // change_y = 0.0;

  // x_new_low = change_x - 64 * RES;
  // x_new_high = change_x + 64 * RES;
  // y_new_low = change_y - 64 * RES;
  // y_new_high = change_y + 64 * RES;

  // if (initialization_phase == false)
  // {

  //   if ((x_new_low >= x_old_low) && (x_old_high >= x_new_low))
  //   {
  //     xstart = x_new_low;
  //     xend = x_old_high;    }

  //   if ((y_new_low >= y_old_low) && (y_old_high >= y_new_low))
  //   {
  //     ystart = y_new_low;
  //     yend = y_old_high;    }

  //   if ((x_new_low < x_old_low) && (x_new_high >= x_old_low))
  //   {
  //     xstart = x_old_low;
  //     xend = x_new_high;    }

  //   if ((y_new_low < y_old_low) && (y_new_high >= y_old_low))
  //   {
  //     ystart = y_old_low;
  //     yend = y_new_high;    }

  //   if ((xstart != -1) && (ystart != -1))
  //   {

  //     /**
  //      * NL = New Low, OH = Old High
  //      */
  //     //x
  //     int index_xNL = find_nearest(GRID_SIZE, xstart, x_new_low, x_new_high, RES);
  //     int index_xNH = find_nearest(GRID_SIZE, xend, x_new_low, x_new_high, RES);
  //     int index_xOL = find_nearest(GRID_SIZE, xstart, x_old_low, x_old_high, RES);
  //     int index_xOH = find_nearest(GRID_SIZE, xend, x_old_low, x_old_high, RES);
  //     //y
  //     int index_yNL = find_nearest(GRID_SIZE, ystart, y_new_low, y_new_high, RES);
  //     int index_yNH = find_nearest(GRID_SIZE, yend, y_new_low, y_new_high, RES);
  //     int index_yOL = find_nearest(GRID_SIZE, ystart, y_old_low, y_old_high, RES);
  //     int index_yOH = find_nearest(GRID_SIZE, yend, y_old_low, y_old_high, RES);

  //     printf("index_xNL: %i, index_xNH: %i, index_xOL: %i, index_xOH: %i\n\n", index_xNL, index_xNH, index_xOL, index_xOH);

  //     for (unsigned int i = 0; i < index_xOH - index_xOL + 1; i++)
  //     {
  //       for (unsigned int j = 0; j < index_yOH - index_yOL + 1; j++)
  //       {
  //         previous_free[index_xNL + i][index_yNL + j] = updated_free[index_xOL + i][index_yOL + j];
  //         previous_occ[index_xNL + i][index_yNL + j] = updated_occ[index_xOL + i][index_yOL + j];
  //       }
  //     }
  //   }
  // }

  // initialization_phase = false;
  // x_old_low = x_new_low;
  // x_old_high = x_new_high;
  // y_old_low = y_new_low;
  // y_old_high = y_new_high;

  //----------END OF CODE------------//
}

/**
 * @brief: Updates current grids with previous grid values plus a decay
 */
void StaticOccupancyNode::mass_update()
{
  for (unsigned int i = 0; i < GRID_SIZE; i++)
  {
    for (unsigned int j = 0; j < GRID_SIZE; j++)
    {
      updated_occP[i][j] = std::min(decay_factor * previous_occ[i][j], 1.0f - previous_free[i][j]);
      updated_freeP[i][j] = std::min(decay_factor * previous_free[i][j], 1.0f - previous_occ[i][j]);
    }
  }
  // Combine measurement and prediction to form posterior occupied and free masses.
  update_of();
}

// updates probabilities using a bayes filter. Takes into account measured probabilities and predicted probability values.
void StaticOccupancyNode::update_of()
{
  for (unsigned int i = 0; i < GRID_SIZE; i++)
  {
    for (unsigned int j = 0; j < GRID_SIZE; j++)
    {
      // probability of cell being unknown
      float unknown_pred = 1.0 - updated_freeP[i][j] - updated_occP[i][j];

      // probabilityy of measured cell being unknown
      float measured_cell_unknown = 1.0 - measured_free[i][j] - measured_occ[i][j];

      // normalizing factor, ensures probabilities for occupancy/free add up to 1
      float k_value = updated_freeP[i][j] * measured_occ[i][j] + updated_occP[i][j] * measured_free[i][j];

      updated_occ[i][j] = (updated_occP[i][j] * measured_cell_unknown + unknown_pred * measured_occ[i][j] + updated_occP[i][j] * measured_occ[i][j]) / (1.0f - k_value);
      updated_free[i][j] = (updated_freeP[i][j] * measured_cell_unknown + unknown_pred * measured_free[i][j] + updated_freeP[i][j] * measured_free[i][j]) / (1.0f - k_value);
    }
  }
}

/**
 * @brief Gets the average of the updated occupancy and updated free values and adds to a cell_probabilities
 *
 * @return Returns a vector of grid cell probabilities used to fill the OccupancyGrid message
 */
std::vector<std::vector<float>> StaticOccupancyNode::getGridCellProbabilities()
{
  std::vector<std::vector<float>> cell_probabilities;
  for (unsigned int i = 0; i < GRID_SIZE; i++)
  {
    std::vector<float> row;
    for (unsigned int j = 0; j < GRID_SIZE; j++)
    {
      float probability = (0.5 * updated_occ[i][j] + 0.5 * (1.0 - updated_free[i][j]));
      row.push_back(probability);
    }
    cell_probabilities.push_back(row);
  }

  return cell_probabilities;
}

int StaticOccupancyNode::find_nearest(int num, float value, float min, float max, float res)
{
  int index = std::floor(num * (value - min + res / 2.) / (max - min + res));
  return index;
}
//------------------------------------------------//

//-------------RAY TRACING HELPERS----------------//
void StaticOccupancyNode::ray_tracing_approximation_y_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive)
{
  int x1 = 0, y1 = 0;

  int slope = 2 * (y2 - y1);
  int slope_error = slope - (x2 - x1);
  int x_sample, y_sample;
  for (int x = x1, y = y1; x < x2; x++)
  {
    // checks if the point is occupied
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

  // if the point ray-traced to is occupied
  if (inclusive == false)
  {
    int x_coordinate = flip_x * x2 + 64;
    int y_coordinate = flip_y * y2 + 64;
    measured_occ[x_coordinate][y_coordinate] = meas_mass;
    measured_free[x_coordinate][y_coordinate] = 0.0;
  }
}

void StaticOccupancyNode::ray_tracing_approximation_x_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive)
{
  int x1 = 0, y1 = 0;

  int slope = 2 * (x2 - x1);
  int slope_error = slope - (y2 - y1);
  int x_sample, y_sample;
  for (int x = x1, y = y1; y < y2; y++)
  {
    // checks if the point is occupied
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

  // if the point ray-traced to is occupied
  if (inclusive == false)
  {
    int x_coordinate = flip_x * x2 + 64;
    int y_coordinate = flip_y * y2 + 64;
    measured_occ[x_coordinate][y_coordinate] = meas_mass;
    measured_free[x_coordinate][y_coordinate] = 0.0;
  }
}

// VERTICLE +
void StaticOccupancyNode::ray_tracing_vertical(int x2)
{
  int x1 = 0;
  int y1 = 0;
  x2 = x2 - 1;

  for (int x = x1; x <= x2; x++)
  {
    // checks if the point is occupied
    if (measured_occ[64][x + 64] == meas_mass)
    {
      printf("BROKE! VERTICAL + \n\n");
      break;
    }

    measured_free[x + 64][64] = meas_mass;
  }

  measured_free[x2 + 64][64] = 0.0;
}

// VERTICLE -
void StaticOccupancyNode::ray_tracing_vertical_n(int x1)
{
  int x2 = 0;
  int y2 = 0;
  x1 = x1 + 1;

  for (int x = x1; x <= x2; x++)
  {
    if (measured_occ[64][x + 64] == meas_mass)
    {
      printf("BROKE! VERTICAL - \n\n");
      break;
    }

    measured_free[x + 64][64] = meas_mass;
  }

  measured_free[x2 + 64][64] = 0.0;
}

// HORIZONTAL +
void StaticOccupancyNode::ray_tracing_horizontal(int y2)
{
  int x1 = 0;
  int y1 = 0;
  y2 = y2 - 1;

  for (int y = y1; y <= y2; y++)
  {
    if (measured_occ[64][y + 64] == meas_mass)
    {
      printf("BROKE! HORIZONTAL + \n\n");
      break;
    }
    measured_free[64][y + 64] = meas_mass;
  }
}

// HORIZONTAL -
void StaticOccupancyNode::ray_tracing_horizontal_n(int y1)
{
  int x1 = 0;
  int y2 = 0;
  y1 = y1 + 1;

  for (int y = y1; y <= y2; y++)
  {
    if (
        measured_occ[64][y + 64] == meas_mass)
    {
      printf("BROKE! HORIZONTAL - \n\n");
      break;
    }
    measured_free[64][y + 64] = meas_mass;
  }

  measured_free[64][y2 + 64] = 0.0;
}

void StaticOccupancyNode::clear()
{
  for (unsigned int i = 0; i < GRID_SIZE; i++)
  {
    for (unsigned int j = 0; j < GRID_SIZE; j++)
    {
      measured_occ[i][j] = 0.0;
      measured_free[i][j] = 0.0;
    }
  }
}
