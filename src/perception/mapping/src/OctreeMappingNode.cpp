/*
 * Package:   mapping
 * Filename:  OctreeMappingNode.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "mapping/OctreeMappingNode.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;

void print_query_info(octomap::point3d query, octomap::OcTreeNode *node)
{
  if (node != NULL)
  {
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  }
  else
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

OctreeMappingNode::OctreeMappingNode() : Node("octree_mapping_node")
{
  RCLCPP_INFO(this->get_logger(), "Hello, world!");

  pcd_sub = this->create_subscription<PointCloud2>("/lidar_filtered", 10, std::bind(&OctreeMappingNode::point_cloud_cb, this, std::placeholders::_1));
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  octomap::OcTree tree(0.1);

  for (int x = -20; x < 20; x++)
  {
    for (int y = -20; y < 20; y++)
    {
      for (int z = -20; z < 20; z++)
      {
        octomap::point3d endpoint((float)x * 0.05f, (float)y * 0.05f, (float)z * 0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x = -30; x < 30; x++)
  {
    for (int y = -30; y < 30; y++)
    {
      for (int z = -30; z < 30; z++)
      {
        octomap::point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f, (float)z * 0.02f - 1.0f);
        tree.updateNode(endpoint, false); // integrate 'free' measurement
      }
    }
  }

  std::cout << std::endl;
  std::cout << "performing some queries:" << std::endl;

  octomap::point3d query(0., 0., 0.);
  octomap::OcTreeNode *result = tree.search(query);
  print_query_info(query, result);

  query = octomap::point3d(-1., -1., -1.);
  result = tree.search(query);
  print_query_info(query, result);

  query = octomap::point3d(1., 1., 1.);
  result = tree.search(query);
  print_query_info(query, result);

  // std::cout << std::endl;
  // tree.writeBinary("simple_tree.bt");
  // std::cout << "wrote example file simple_tree.bt" << std::endl
  //           << std::endl;
  // std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
  // std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl
  //           << std::endl;
}

void OctreeMappingNode::point_cloud_cb(PointCloud2::SharedPtr ros_cloud)
{
  octomap::Pointcloud octo_cloud;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Convert from ROS to PCL format
  pcl::fromROSMsg(*ros_cloud, pcl_cloud);

  // Transform from base_link->map
  TransformStamped t;

  clock.clock.nanosec -= 1e8;

  try
  {
    t = tf_buffer_->lookupTransform(
        "map", ros_cloud->header.frame_id,
        this->clock.clock);
  }
  catch (const tf2::TransformException &ex)
  {
    // This warning will be called no more than once every 5 seconds
    // https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html#a451bee77c253ec72f4984bb577ff818a
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Could not transform %s to %s: %s",
        "map", ros_cloud->header.frame_id.c_str(), ex.what());
    return;
  }

  // Convert the TransformStamped message that we just received
  // into an Eigen-style transform
  Eigen::Matrix4f baselink_to_map_tf = tf2::transformToEigen(t.transform).matrix().cast<float>();

  // Use the Eigen transform to transform the pcl cloud to the global "map" frame
  pcl::transformPointCloud(pcl_cloud, pcl_cloud, baselink_to_map_tf);

  Vector3 tf_translation = t.transform.translation;
  octomap::point3d sensor_origin(
      tf_translation.x, tf_translation.y, tf_translation.z);

  tree.insertPointCloud(pclToOctreeCloud(pcl_cloud), sensor_origin);
}

octomap::Pointcloud OctreeMappingNode::pclToOctreeCloud(pcl::PointCloud<pcl::PointXYZI> inputCloud)
{
  octomap::Pointcloud result;
  for (pcl::PointXYZI pt : inputCloud)
  {
    // RCLCPP_INFO(this->get_logger(), std::to_string(pt.x).c_str());
    result.push_back(pt.x, pt.y, pt.z);
  }
  RCLCPP_INFO(this->get_logger(), "Adding %d points", inputCloud.size());
  return result;
}

OctreeMappingNode::~OctreeMappingNode()
{
  RCLCPP_INFO(this->get_logger(), "Saving map.");
  tree.writeBinary("simple_tree.bt");
}
