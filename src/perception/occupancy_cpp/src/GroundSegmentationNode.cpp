/*
 * Package:   occupancy_cpp
 * Filename:  GroundSegmentationNode.cpp
 * Author:    Will Heitman, Daniel Vayman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "occupancy_cpp/GroundSegmentationNode.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

GroundSegmentationNode::GroundSegmentationNode() : Node("ground_segmentation_node")
{
  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  raw_lidar_sub = this->create_subscription<PointCloud2>(
      "/carla/hero/lidar", 10,
      std::bind(&GroundSegmentationNode::pointCloudCb, this, std::placeholders::_1));

  filtered_lidar_pub = this->create_publisher<PointCloud2>("/lidar/filtered", 10);
}

void GroundSegmentationNode::pointCloudCb(PointCloud2::SharedPtr raw_cloud)
{
  RCLCPP_INFO(this->get_logger(), "Points for the segmentation received.");
  auto now = this->get_clock()->now();
  float start_time = now.seconds() + now.nanoseconds() * 9.0 ; // Get current (start) time in seconds

  /* Parameters that can be specified via class later. */

  // Specify the vertical offset distance of the LiDAR from the ground (specified by system - e.g. KITTI).
  float lidarZ = 0.0;

  // Maximum keep LiDAR square distance.
  float maxKeep = 80.0;

  float s = 0.55; // 0.6; // 0.09
  float res = 0.4;

  /* Build the grid. */

  // Initialize the filtered point cloud to be published.
  PointCloud2 result_cloud;

  // Set the message time stamp.
  result_cloud.header.stamp = raw_cloud->header.stamp;
  // std::cout << "time stamp" << result_cloud.header.stamp << endl;

  // Dereference the point cloud.
  const PointCloud2 &cloud(*raw_cloud);

  // Convert from ROS to PCL format
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*raw_cloud, pcl_cloud);

  // Get number of points in point cloud.
  int origPCSize = raw_cloud->width;

  // Get the maximum x, y distance.
  float maxXYCoord = maxKeep;

  // max( max(-1 * minPoint.x, maxPoint.x), max(-1 * minPoint.y, maxPoint.y));

  size_t gridSize = int(2 * ceil((maxXYCoord) / res) + 1);

  std::vector<int> grid[gridSize][gridSize];

  // Get center coordinates of the grid.
  int centerX = int(ceil(maxXYCoord / res));
  int centerY = int(ceil(maxXYCoord / res));

  // Populate the grid with indices of points that fit into a particular cell.
  for (pcl::PointXYZI point : pcl_cloud)
  {
    // Ensure within specified area, and not above a filtered height
    if ((abs(point.x) <= maxXYCoord) && (abs(point.y) <= maxXYCoord) && (point.z <= 3.5))
    {
      grid[int(centerX + round(point.x / res))][int(centerY + round(point.y / res))].push_back(i);
      // cout << "Z value: " << point.z << endl;
    }
  }

  // /* Perform MRF segmentation. */

  // // Initialize the hG array.
  // float hG[gridSize][gridSize];

  // // Initialize the grid segmentation (ground cells have a value of 1).
  // int gridSeg[gridSize][gridSize];
  // fill(gridSeg[0], gridSeg[0] + gridSize * gridSize, 0);

  // // Initialize the center coordinate of the 2D grid to ground according to the height of the
  // // LiDAR position on the vehicle.
  // hG[centerX][centerY] = -1 * lidarZ;
  // gridSeg[centerX][centerY] = 1;

  // // Allocate space for two elements in the vector.
  // vector<int> outerIndex;
  // outerIndex.resize(2);

  // // Move radially outwards and perform the MRF segmentation.
  // for (int i = 1; i < int(ceil(maxXYCoord / res)) + 1; i++)
  // {

  //   // Generate the indices at the ith circle level from the center of the grid.
  //   outerIndex[0] = -1 * i;
  //   outerIndex[1] = i;

  //   /* vector<int> outerIndex;
  //   outerIndex.push_back(-1 * i);
  //   outerIndex.push_back(i); */

  //   for (int index : outerIndex)
  //   {
  //     for (int k = -1 * i; k < (i + 1); k++)
  //     {

  //       // Index is the outer index (perimeter of circle) and k is possible inner indices
  //       // in between the edges.
  //       Indices currentCircle;
  //       currentCircle.insert(pair<int, int>(centerX + index, centerY + k));

  //       // Add the mirror image of cells to the circle if not on the top or bottom row of the circle.
  //       if (!((k == -1 * i) || (k == i)))
  //       {
  //         currentCircle.insert(pair<int, int>(centerX + k, centerY + index));
  //       }

  //       // Go through the one or two stored cells right now.
  //       for (pair<int, int> const &indexPair : currentCircle)
  //       {

  //         int x = indexPair.first;
  //         int y = indexPair.second;

  //         // Compute the minimum and maximum z coordinates of each grid cell.

  //         // Initialize H to a very small value.
  //         float H = -numeric_limits<float>::infinity();
  //         // Initialize h to a very large value.
  //         float h = numeric_limits<float>::infinity();

  //         if (!grid[x][y].empty())
  //         {

  //           const vector<int> pcIndices = grid[x][y];

  //           for (int j = 0; j < pcIndices.size(); j++)
  //           {
  //             H = max(cloud.points[pcIndices[j]].z, H);
  //             h = min(cloud.points[pcIndices[j]].z, h);
  //           }
  //         }

  //         // Pay attention to what happens when there are no points in a grid cell? Will it work?

  //         // Compute hHatG: find max hG of neighbors.
  //         float hHatG = -numeric_limits<float>::infinity();

  //         // Get the inner circle neighbors of the current cell.

  //         // The inner circle is one level down from the current circle.
  //         int innerCircleIndex = i - 1;

  //         // Center the grid at (0, 0).
  //         int xRelativeIndex = x - centerX;
  //         int yRelativeIndex = y - centerY;

  //         // Loop through possible neighbor indices.
  //         for (int m = -1; m < 2; m++)
  //         {
  //           for (int n = -1; n < 2; n++)
  //           {

  //             int xRelativeNew = abs(xRelativeIndex + m);
  //             int yRelativeNew = abs(yRelativeIndex + n);

  //             // Ensure index is actually on the inner circle.
  //             if (((xRelativeNew == innerCircleIndex) && (yRelativeNew <= innerCircleIndex)) || ((yRelativeNew == innerCircleIndex) && (xRelativeNew <= innerCircleIndex)))
  //             {

  //               // Compute the new hHatG.
  //               float hGTemp = hG[x + m][y + n];
  //               hHatG = max(hGTemp, hHatG);
  //             }
  //           }
  //         }

  //         // Update hG of current cell.
  //         if ((H != -numeric_limits<float>::infinity()) && (h != numeric_limits<float>::infinity()) &&
  //             ((H - h) < s) && ((H - hHatG) < s))
  //         {
  //           gridSeg[x][y] = 1;
  //           hG[x][y] = H;
  //         }
  //         else
  //         {
  //           hG[x][y] = hHatG;

  //           // Add the cell's LiDAR points to the segmented (not ground) point cloud.
  //           if (!grid[x][y].empty())
  //           {

  //             const vector<int> pcIndices = grid[x][y];

  //             for (int j = 0; j < grid[x][y].size(); j++)
  //             {
  //               result_cloud.points.push_back(cloud.points[pcIndices[j]]);
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  // auto now = this->get_clock()->now();
  // float end_time = now.seconds() + now.nanoseconds() * 9.0 ; // Get current (start) time in seconds
  // // cout << "Width of point cloud before: " << cloud.width << endl;
  // // cout << "Width of point cloud now: " << result_cloud.points.size() << endl;
  // RCLCPP_INFO(this->get_logger(), "Ground seg took %d", end_time - start_time);

  // /* Set the frame_id to "vehicle_ground_cartesian". */

  // /* Publish to ROS node. */
  // result_cloud.header.frame_id = cloud.header.frame_id;
  filtered_lidar_pub->publish(result_cloud);
}