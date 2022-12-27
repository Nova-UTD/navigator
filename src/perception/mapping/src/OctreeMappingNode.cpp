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

using carla_msgs::msg::CarlaWorldInfo;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;

OctreeMappingNode::OctreeMappingNode() : Node("octree_mapping_node")
{
  RCLCPP_INFO(this->get_logger(), "Hello, world!");

  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  pcd_sub = this->create_subscription<PointCloud2>("/lidar_filtered", 10, std::bind(&OctreeMappingNode::pointCloudCb, this, std::placeholders::_1));

  world_info_sub = this->create_subscription<CarlaWorldInfo>(
      "/carla/world_info", 10,
      std::bind(&OctreeMappingNode::worldInfoCb, this, std::placeholders::_1));

  this->map_marker_timer = this->create_wall_timer(this->MAP_UPDATE_PERIOD,
                                                   bind(&OctreeMappingNode::publishMapMarker, this));
}

void OctreeMappingNode::publishMapMarker()
{
  RCLCPP_INFO(this->get_logger(), "Updating map.");
}

void OctreeMappingNode::pointCloudCb(PointCloud2::SharedPtr ros_cloud)
{
  if (this->tree == nullptr)
    return; // Tree not yet initialized
  RCLCPP_INFO(this->get_logger(), "PCD received!");

  octomap::Pointcloud octo_cloud;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Convert from ROS to PCL format
  pcl::fromROSMsg(*ros_cloud, pcl_cloud);

  // Transform from base_link->map
  TransformStamped t;

  // Avoid "future time" error from TF2 due to slight delay
  // in incoming LiDAR data
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

  tree->insertPointCloud(pclToOctreeCloud(pcl_cloud), sensor_origin);
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

std::string OctreeMappingNode::getFilenameFromMapName()
{
  std::string file_name = this->map_name;
  file_name.erase(
      remove(file_name.begin(), file_name.end(), '/'),
      file_name.end()); // remove '/' from string

  std::size_t ind = file_name.find("CarlaMaps"); // Remove "CarlaMaps" from name
  if (ind != std::string::npos)
  {
    file_name.erase(ind, std::string("CarlaMaps").length());
    std::cout << file_name << "\n";
  }

  std::transform(file_name.begin(), file_name.end(),
                 file_name.begin(), ::tolower); // Convert to lowercase

  file_name += ".bt";
  return file_name;
}

void OctreeMappingNode::saveOctreeBinary()
{
  std::string file_name = getFilenameFromMapName();
  RCLCPP_INFO(this->get_logger(), "Saving map to " + file_name);

  tree->writeBinary(file_name);
}

void OctreeMappingNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "World info cb");
  if (this->tree != nullptr)
    return; // Tree already initialized

  this->map_name = msg->map_name;
  std::string file_name = getFilenameFromMapName();
  RCLCPP_INFO(this->get_logger(), "Reading map from " + file_name);

  this->tree = std::make_shared<octomap::OcTree>(OCTREE_RESOLUTION);

  bool successfully_read = this->tree->readBinary(file_name);

  if (!successfully_read)
  {
    this->tree = std::make_shared<octomap::OcTree>(this->OCTREE_RESOLUTION);
    RCLCPP_INFO(this->get_logger(), "Map file did not exist. A new one will be created.");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Reading complete.");
  }
}

OctreeMappingNode::~OctreeMappingNode()
{
  saveOctreeBinary();
}
