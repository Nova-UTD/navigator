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
using geometry_msgs::msg::Point;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

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

  voxel_marker_pub = this->create_publisher<Marker>("/map/voxels/viz", 10);

  this->map_marker_timer = this->create_wall_timer(this->MAP_UPDATE_PERIOD,
                                                   bind(&OctreeMappingNode::publishMapMarker, this));
}

void OctreeMappingNode::publishMapMarker()
{
  // If the tree is uninitialized, skip
  if (this->tree == nullptr)
    return;

  // Set up a Marker message that we can add our voxels to
  Marker voxel_list;
  voxel_list.header.frame_id = "map";
  voxel_list.header.stamp = this->clock.clock;
  voxel_list.ns = "voxel_map";
  voxel_list.action = voxel_list.ADD;
  voxel_list.type = voxel_list.CUBE_LIST;
  ColorRGBA voxel_color;
  voxel_color.r = 1.0;
  voxel_color.b = 1.0;
  voxel_color.a = 0.5;
  Vector3 scale;
  scale.x = 0.8;
  scale.y = 0.8;
  scale.z = 0.8;
  voxel_list.scale = scale;
  voxel_list.color = voxel_color;
  voxel_list.frame_locked = true;

  // This is the current location of our vehicle, set by lidarCb()
  Vector3 center_ros = map_bl_transform.transform.translation;

  // Set the bounds of our bounding box
  octomap::point3d hi_res_bbx_min_pt(
      center_ros.x - MED_RES_DISTANCE,
      center_ros.y - MED_RES_DISTANCE,
      center_ros.z - MED_RES_DISTANCE);
  octomap::point3d hi_res_bbx_max_pt(
      center_ros.x + MED_RES_DISTANCE,
      center_ros.y + MED_RES_DISTANCE,
      center_ros.z + MED_RES_DISTANCE);

  // Iterate through each point in the tree
  for (
      octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(hi_res_bbx_min_pt, hi_res_bbx_max_pt, 14),
                                         end = tree->end_leafs_bbx();
      it != end; ++it)
  {

    if (it->getOccupancy() < 0.8)
      continue; // Skip unoccupied cells
    RCLCPP_INFO(this->get_logger(), "Occ: %f, val: %f", it->getOccupancy(), it->getValue());
    // manipulate node, e.g.:44
    octomap::point3d voxel_center = it.getCoordinate();
    Point voxel_center_msg;
    voxel_center_msg.x = voxel_center.x();
    voxel_center_msg.y = voxel_center.y();
    voxel_center_msg.z = voxel_center.z();
    voxel_list.points.push_back(voxel_center_msg);
  }
  voxel_marker_pub->publish(voxel_list);
}

void OctreeMappingNode::pointCloudCb(PointCloud2::SharedPtr ros_cloud)
{
  if (this->tree == nullptr)
    return; // Tree not yet initialized

  octomap::Pointcloud octo_cloud;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Convert from ROS to PCL format
  pcl::fromROSMsg(*ros_cloud, pcl_cloud);

  // Avoid "future time" error from TF2 due to slight delay
  // in incoming LiDAR data
  clock.clock.nanosec -= 1e8;

  // Transform from base_link->map
  try
  {
    map_bl_transform = tf_buffer_->lookupTransform(
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
  Eigen::Matrix4f baselink_to_map_tf = tf2::transformToEigen(map_bl_transform.transform).matrix().cast<float>();

  // Use the Eigen transform to transform the pcl cloud to the global "map" frame
  pcl::transformPointCloud(pcl_cloud, pcl_cloud, baselink_to_map_tf);

  Vector3 tf_translation = map_bl_transform.transform.translation;
  octomap::point3d sensor_origin(
      tf_translation.x, tf_translation.y, tf_translation.z);

  tree->insertPointCloud(pclToOctreeCloud(pcl_cloud), sensor_origin);
}

/**
 * @brief Given a PCL-formatted cloud, return an Octomap-formatted version
 *
 * @param inputCloud The PCL-formatted cloud
 * @return octomap::Pointcloud
 */
octomap::Pointcloud OctreeMappingNode::pclToOctreeCloud(pcl::PointCloud<pcl::PointXYZI> inputCloud)
{
  octomap::Pointcloud result;
  for (pcl::PointXYZI pt : inputCloud)
  {
    result.push_back(pt.x, pt.y, pt.z);
  }
  RCLCPP_INFO(this->get_logger(), "Adding %d points", inputCloud.size());
  return result;
}

/**
 * @brief Generate the map's filename, including the path
 *
 * @return std::string
 */
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

  file_name = MAP_SAVE_PATH + file_name + ".bt";
  return file_name;
}

/**
 * @brief Save the octree to disk in the Octomap format.
 *
 */
void OctreeMappingNode::saveOctreeBinary()
{
  std::string file_name = getFilenameFromMapName();
  RCLCPP_INFO(this->get_logger(), "Saving map to " + file_name);

  tree->writeBinary(file_name);
}

/**
 * @brief Either load or create a map from a given map name
 *
 * @param msg The world info message containing the map name
 */
void OctreeMappingNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
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

/**
 * @brief Destroy the Octree node, saving the map to disk first.
 *
 */
OctreeMappingNode::~OctreeMappingNode()
{
  saveOctreeBinary();
}
