/*
 * Package:   mapping
 * Filename:  OctoSlamNode.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "mapping/OctoSlamNode.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

using carla_msgs::msg::CarlaWorldInfo;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

struct Particle
{
  double x;
  double y;
  double theta;
  double weight;
};

OctoSlamNode::OctoSlamNode() : Node("octree_mapping_node")
{
  // Subscribe to and use CARLA's clock
  clock_sub = this->create_subscription<Clock>(
      "/clock", 10,
      [this](Clock::SharedPtr msg)
      { this->clock = *msg; });

  pcd_sub = this->create_subscription<PointCloud2>(
      "/lidar_filtered", 10,
      std::bind(&OctoSlamNode::pointCloudCb, this, std::placeholders::_1));

  initial_odom_sub = this->create_subscription<Odometry>(
      INITIAL_GUESS_ODOM_TOPIC, 10,
      std::bind(&OctoSlamNode::gnssOdomCb, this, std::placeholders::_1));

  world_info_sub = this->create_subscription<CarlaWorldInfo>(
      "/carla/world_info", 10,
      std::bind(&OctoSlamNode::worldInfoCb, this, std::placeholders::_1));

  voxel_marker_pub = this->create_publisher<PointCloud2>("/map/voxels/viz", 10);
  particle_viz_pub = this->create_publisher<PointCloud2>("/map/particles/viz", 10);

  this->map_marker_timer = this->create_wall_timer(this->MAP_UPDATE_PERIOD,
                                                   bind(&OctoSlamNode::publishMapMarker, this));

  this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

double quatToYaw(double w, double x, double y, double z)
{
  double t1 = 2.0 * (w * z + x * y);
  double t2 = 1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(t1, t2);

  return yaw;
}

void OctoSlamNode::gnssOdomCb(Odometry::SharedPtr msg)
{
  // Initialize our filter given the initial guess
  if (this->filter == nullptr)
  {
    if (this->tree == nullptr)
      return;
    PoseWithCovarianceStamped initial_guess;
    initial_guess.pose = msg->pose;
    initial_guess.header = msg->header;
    this->filter = std::make_shared<ParticleFilter>(initial_guess, 1000, *this->tree);
    RCLCPP_INFO(this->get_logger(), "Particle filter has been created.");
  }

  // Extract yaw from msg quaternion
  auto q = msg->pose.pose.orientation;
  double yaw = quatToYaw(q.w, q.x, q.y, q.z);

  navigator::perception::Pose gnss_pose(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yaw);

  // filter->update() runs one iteration of the particle filter,
  // handling all internal steps, then returns the result pose.
  PoseWithCovarianceStamped filter_result = this->filter->update(this->latest_cloud, gnss_pose);
  RCLCPP_INFO(this->get_logger(), "097");

  map_bl_transform = TransformStamped();

  map_bl_transform.header.stamp = this->clock.clock;
  map_bl_transform.header.frame_id = "map";
  map_bl_transform.child_frame_id = "hero";
  map_bl_transform.transform.translation.x = filter_result.pose.pose.position.x;
  map_bl_transform.transform.translation.y = filter_result.pose.pose.position.y;
  map_bl_transform.transform.translation.z = filter_result.pose.pose.position.z; // This should be zero.
  map_bl_transform.transform.rotation = filter_result.pose.pose.orientation;
  this->tf_broadcaster->sendTransform(map_bl_transform);

  PointCloud2 particle_cloud = this->filter->asPointCloud();
  particle_viz_pub->publish(particle_cloud);
}

void OctoSlamNode::publishMapMarker()
{
  // If the tree is uninitialized, skip
  if (this->tree == nullptr)
    return;

  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Convert from ROS to PCL format

  // Location of vehicle as a ROS vector
  Vector3 center_ros = map_bl_transform.transform.translation;

  // Set the bounds of our bounding box
  octomap::point3d hi_res_bbx_min_pt(
      center_ros.x - HIGH_RES_DISTANCE,
      center_ros.y - HIGH_RES_DISTANCE,
      center_ros.z - HIGH_RES_DISTANCE);
  octomap::point3d hi_res_bbx_max_pt(
      center_ros.x + HIGH_RES_DISTANCE,
      center_ros.y + HIGH_RES_DISTANCE,
      center_ros.z + HIGH_RES_DISTANCE);

  // Iterate through each point in the tree
  for (
      octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(hi_res_bbx_min_pt, hi_res_bbx_max_pt, MAX_VISUALIZATION_DEPTH),
                                         end = tree->end_leafs_bbx();
      it != end; ++it)
  {

    if (it->getOccupancy() < 0.8)
      continue; // Skip unoccupied cells
    // RCLCPP_INFO(this->get_logger(), "Occ: %f, val: %f", it->getOccupancy(), it->getValue());
    // manipulate node, e.g.:44
    octomap::point3d voxel_center = it.getCoordinate();
    pcl::PointXYZI voxel_center_pcl;
    voxel_center_pcl.x = voxel_center.x();
    voxel_center_pcl.y = voxel_center.y();
    voxel_center_pcl.z = voxel_center.z();

    pcl_cloud.push_back(voxel_center_pcl);
  }

  PointCloud2 ros_cloud;

  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = "map";
  ros_cloud.header.stamp = this->clock.clock;

  voxel_marker_pub->publish(ros_cloud);
}

void OctoSlamNode::pointCloudCb(PointCloud2::SharedPtr ros_cloud)
{
  auto now = this->get_clock()->now();
  double start_seconds = now.seconds() + now.nanoseconds() * 1e-9;

  if (this->tree == nullptr)
    return; // Tree not yet initialized

  octomap::Pointcloud octo_cloud;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Convert from ROS to PCL format
  pcl::fromROSMsg(*ros_cloud, pcl_cloud);

  // Convert the TransformStamped message that we just received
  // into an Eigen-style transform
  Eigen::Matrix4f baselink_to_map_tf = tf2::transformToEigen(map_bl_transform.transform).matrix().cast<float>();

  // Use the Eigen transform to transform the pcl cloud to the global "map" frame
  pcl::transformPointCloud(pcl_cloud, pcl_cloud, baselink_to_map_tf);

  Vector3 tf_translation = map_bl_transform.transform.translation;
  octomap::point3d sensor_origin(
      tf_translation.x, tf_translation.y, tf_translation.z);

  tree->insertPointCloud(pclToOctreeCloud(pcl_cloud), sensor_origin);
  this->latest_cloud = pcl_cloud; // Cache the latest LiDAR data for the particle filter
  now = this->get_clock()->now();
  double end_seconds = now.seconds() + now.nanoseconds() * 1e-9;

  double delta_t = end_seconds - start_seconds;
  RCLCPP_INFO(this->get_logger(), "Took %f seconds", delta_t);
}

/**
 * @brief Given a PCL-formatted cloud, return an Octomap-formatted version
 *
 * @param inputCloud The PCL-formatted cloud
 * @return octomap::Pointcloud
 */
octomap::Pointcloud OctoSlamNode::pclToOctreeCloud(pcl::PointCloud<pcl::PointXYZI> inputCloud)
{
  octomap::Pointcloud result;
  for (pcl::PointXYZI pt : inputCloud)
  {
    result.push_back(pt.x, pt.y, pt.z);
  }
  RCLCPP_DEBUG(this->get_logger(), "Adding %d points", inputCloud.size());
  return result;
}

/**
 * @brief Generate the map's filename, including the path
 *
 * @return std::string
 */
std::string OctoSlamNode::getFilenameFromMapName()
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
void OctoSlamNode::saveOctreeBinary()
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
void OctoSlamNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
  if (this->tree != nullptr)
    return; // Tree already initialized

  this->map_name = msg->map_name;
  std::string file_name = getFilenameFromMapName();

  // if (file_name == ".bt")
  // {
  //   RCLCPP_WARN(this->get_logger(), "Map name is empty, waiting until map is loaded." + file_name);
  // }

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
OctoSlamNode::~OctoSlamNode()
{
  saveOctreeBinary();
}
