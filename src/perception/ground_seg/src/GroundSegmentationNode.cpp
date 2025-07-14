/*
 * Package:   occupancy_cpp
 * Filename:  GroundSegmentationNode.cpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "ground_seg/GroundSegmentationNode.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

GroundSegmentationNode::GroundSegmentationNode() : Node("ground_segmentation_node")
{
  this->declare_parameter<double>("sensitivity", 0.2);

  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  raw_lidar_sub = this->create_subscription<PointCloud2>(
      "/lidar", 10,
      std::bind(&GroundSegmentationNode::pointCloudCb, this, std::placeholders::_1));

  filtered_lidar_pub = this->create_publisher<PointCloud2>("/lidar/filtered", 10);
}

/**
 * @brief Each time that raw LiDAR is received, remove the ground points and publish the filtered version.
 *
 * @param raw_cloud The unfiltered LiDAR point cloud
 */
void GroundSegmentationNode::pointCloudCb(PointCloud2::SharedPtr msg)
{
  PointCloud2 filtered_msg;

  pcl::PointCloud<pcl::PointXYZI> raw_cloud;
  pcl::fromROSMsg(*msg, raw_cloud); // Convert the incoming message to PCL format

  pcl::PointCloud<pcl::PointXYZI> filtered_cloud = removeGround(raw_cloud);

  pcl::toROSMsg(filtered_cloud, filtered_msg);

  filtered_msg.header = msg->header;

  filtered_lidar_pub->publish(filtered_msg);
}

/**
 * @brief Remove ground points using Markov Random Field method
 * https://ieeexplore.ieee.org/document/9410344
 *
 * @param raw_cloud
 * @return pcl::PointCloud<pcl::PointXYZI>
 */
pcl::PointCloud<pcl::PointXYZI> GroundSegmentationNode::removeGround(pcl::PointCloud<pcl::PointXYZI> raw_cloud)
{
  float lidar_height = 0.0; // TODO: Make ROS parameter
  float range = 80.0;
  float s = this->get_parameter("sensitivity").as_double();         // Original: 0.55
  float res = 0.4;        // Grid cell size, in meters
  float max_height = 2.5; // Exclude points above this height, in meters

  pcl::PointCloud<pcl::PointXYZI> filtered_pcd;

  size_t grid_size = int(2 * ceil((range) / res) + 1);
  std::vector<int> grid[grid_size][grid_size];

  int center_x = int(ceil(range / res));
  int center_y = int(ceil(range / res));

  // Populate the grid with indices of points that fit into a particular cell.
  for (int i = 0; i < raw_cloud.size(); i++)
  {
    pcl::PointXYZI point = raw_cloud.at(i);

    // If the point is within range, add the point's index to the respective cell
    if ((abs(point.x) <= range) && (abs(point.y) <= range) && (point.z <= max_height))
      grid[int(center_x + round(point.x / res))][int(center_y + round(point.y / res))].push_back(i);
  }

  // Initialize the hG array.
  float hG[grid_size][grid_size];

  // Initialize the grid segmentation (ground cells have a value of 1).
  int gridSeg[grid_size][grid_size];
  std::fill(gridSeg[0], gridSeg[0] + grid_size * grid_size, 0);

  // Initialize the center coordinate of the 2D grid to ground according to the height of the
  // LiDAR position on the vehicle.
  hG[center_x][center_y] = -1 * lidar_height;
  gridSeg[center_x][center_y] = 1;

  // Allocate space for two elements in the vector.
  std::vector<int> outerIndex;
  outerIndex.resize(2);

  // Move radially outwards and perform the MRF segmentation.
  for (int i = 1; i < int(ceil(range / res)) + 1; i++)
  {

    // Generate the indices at the ith circle level from the center of the grid.
    outerIndex[0] = -1 * i;
    outerIndex[1] = i;

    for (int index : outerIndex)
    {
      for (int k = -1 * i; k < (i + 1); k++)
      {

        // Index is the outer index (perimeter of circle) and k is possible inner indices
        // in between the edges.
        std::set<std::pair<int,int>> currentCircle;
        currentCircle.insert(std::pair<int, int>(center_x + index, center_y + k));

        // Add the mirror image of cells to the circle if not on the top or bottom row of the circle.
        if (!((k == -1 * i) || (k == i)))
        {
          currentCircle.insert(std::pair<int, int>(center_x + k, center_y + index));
        }

        // Go through the one or two stored cells right now.
        for (std::pair<int, int> const &indexPair : currentCircle)
        {

          int x = indexPair.first;
          int y = indexPair.second;

          // Compute the minimum and maximum z coordinates of each grid cell.

          // Initialize H to a very small value.
          float H = -std::numeric_limits<float>::infinity();
          // Initialize h to a very large value.
          float h = std::numeric_limits<float>::infinity();

          if (!grid[x][y].empty())
          {

            const std::vector<int> pcIndices = grid[x][y];

            for (int j = 0; j < pcIndices.size(); j++)
            {
              H = std::max(raw_cloud.at(pcIndices[j]).z, H);
              h = std::min(raw_cloud.at(pcIndices[j]).z, h);
            }
          }

          // Pay attention to what happens when there are no points in a grid cell? Will it work?

          // Compute hHatG: find max hG of neighbors.
          float hHatG = -std::numeric_limits<float>::infinity();

          // Get the inner circle neighbors of the current cell.

          // The inner circle is one level down from the current circle.
          int innerCircleIndex = i - 1;

          // Center the grid at (0, 0).
          int xRelativeIndex = x - center_x;
          int yRelativeIndex = y - center_y;

          // Loop through possible neighbor indices.
          for (int m = -1; m < 2; m++)
          {
            for (int n = -1; n < 2; n++)
            {

              int xRelativeNew = abs(xRelativeIndex + m);
              int yRelativeNew = abs(yRelativeIndex + n);

              // Ensure index is actually on the inner circle.
              if (((xRelativeNew == innerCircleIndex) && (yRelativeNew <= innerCircleIndex)) || ((yRelativeNew == innerCircleIndex) && (xRelativeNew <= innerCircleIndex)))
              {

                // Compute the new hHatG.
                float hGTemp = hG[x + m][y + n];
                hHatG = std::max(hGTemp, hHatG);
              }
            }
          }

          // Update hG of current cell.
          if ((H != -std::numeric_limits<float>::infinity()) && (h != std::numeric_limits<float>::infinity()) &&
              ((H - h) < s) && ((H - hHatG) < s))
          {
            gridSeg[x][y] = 1;
            hG[x][y] = H;
          }
          else
          {
            hG[x][y] = hHatG;

            // Add the cell's LiDAR points to the segmented (not ground) point cloud.
            if (!grid[x][y].empty())
            {

              const std::vector<int> pcIndices = grid[x][y];

              for (int j = 0; j < grid[x][y].size(); j++)
              {
                filtered_pcd.push_back(raw_cloud.at(pcIndices[j]));
              }
            }
          }
        }
      }
    }
  }

  return filtered_pcd;
}

GroundSegmentationNode::~GroundSegmentationNode()
{
  // Do nothing for now.
}
