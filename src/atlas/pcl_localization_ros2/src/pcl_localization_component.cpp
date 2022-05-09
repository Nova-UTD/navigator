#include <pcl_localization/pcl_localization_component.hpp>
#include <exception>

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
      broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("trans_epsilon", 0.01);
  declare_parameter("max_iterations", 35);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("enable_logging_to_file", false);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("fitness_gnss_threshold", 10.0);
  declare_parameter("dist_gnss_threshold", 5.0);
  declare_parameter("pose_position_cov_k", 1.0);
  declare_parameter("ndt_num_threads", 4);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  // path_.header.frame_id = global_frame_id_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_)
  {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    // path_.poses.push_back(*msg);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);

    if (registration_method_ == "GICP")
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    }
    else
    {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();

  return CallbackReturn::SUCCESS;
  csvFile->close();
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  
  return CallbackReturn::SUCCESS;
  
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State &state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("trans_epsilon", trans_epsilon_);
  get_parameter("max_iterations", max_iterations_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("enable_logging_to_file", enable_logging_to_file_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("fitness_gnss_threshold", fitness_gnss_threshold);
  get_parameter("dist_gnss_threshold", dist_gnss_threshold);
  get_parameter("pose_position_cov_k", pose_position_cov_k);
  get_parameter("ndt_num_threads", ndt_num_threads);
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "path",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "initial_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", 10,
      std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  // TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP")
  {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp(
        new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(trans_epsilon_);
    registration_ = gicp;
  }
  else
  {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(trans_epsilon_);
    ndt->setMaximumIterations(max_iterations_);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  }

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

void PCLLocalization::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match　global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  current_pose_stamped_ = *msg;
  pose_pub_->publish(current_pose_stamped_);
}

void PCLLocalization::mapReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP")
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
  }
  else
  {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
}

void PCLLocalization::odomReceived(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_)
  {
    return;
  }
  auto gps_pos = msg->pose.pose.position;
  auto pcd_pos = current_pose_stamped_.pose.pose.position;
  bool fitnessOk = latest_fitness_score < fitness_gnss_threshold;

  if (initialpose_recieved_ == false) {
    RCLCPP_INFO(get_logger(), "Initial pose set from GNSS.");
    current_pose_stamped_.pose.pose = msg->pose.pose;
    current_pose_stamped_.header = msg->header;
    initialpose_recieved_ = true;
  }

  // double current_odom_received_time = msg->header.stamp.sec +
  //                                     msg->header.stamp.nanosec * 1e-9;
  // double dt_odom = current_odom_received_time - last_odom_received_time_;
  // last_odom_received_time_ = current_odom_received_time;
  // if (dt_odom > 1.0 /* [sec] */)
  // {
  //   RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
  //   return;
  // }
  // if (dt_odom < 0.0 /* [sec] */)
  // {
  //   RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
  //   return;
  // }

  // tf2::Quaternion previous_quat_tf;
  // double roll, pitch, yaw;
  // tf2::fromMsg(corrent_pose_stamped_.pose.orientation, previous_quat_tf);
  // tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  // roll += msg->twist.twist.angular.x * dt_odom;
  // pitch += msg->twist.twist.angular.y * dt_odom;
  // yaw += msg->twist.twist.angular.z * dt_odom;

  // Eigen::Quaterniond quat_eig =
  //     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
  //     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  // geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  // Eigen::Vector3d odom{
  //     msg->twist.twist.linear.x,
  //     msg->twist.twist.linear.y,
  //     msg->twist.twist.linear.z};
  // Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  // corrent_pose_stamped_.pose.position.x += delta_position.x();
  // corrent_pose_stamped_.pose.position.y += delta_position.y();
  // corrent_pose_stamped_.pose.position.z += delta_position.z();
  // corrent_pose_stamped_.pose.orientation = quat_msg;

  if (!fitnessOk) {
    current_pose_stamped_.pose.pose = msg->pose.pose;
    current_pose_stamped_.header = msg->header;
    RCLCPP_INFO(get_logger(), "Fitness score was above threshold, adding GPS");
  }

}

void PCLLocalization::imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_)
  {
    return;
  }

  // double roll, pitch, yaw;
  // tf2::Quaternion orientation;
  // tf2::fromMsg(msg->orientation, orientation);
  // tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  // float acc_x = msg->linear_acceleration.x + sin(pitch) * 9.81;
  // float acc_y = msg->linear_acceleration.y - cos(pitch) * sin(roll) * 9.81;
  // float acc_z = msg->linear_acceleration.z - cos(pitch) * cos(roll) * 9.81;

  // Eigen::Vector3f angular_velo{msg->angular_velocity.x, msg->angular_velocity.y,
  //                              msg->angular_velocity.z};
  // Eigen::Vector3f acc{acc_x, acc_y, acc_z};
  // Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
  //                         msg->orientation.z};
  // double imu_time = msg->header.stamp.sec +
  //                   msg->header.stamp.nanosec * 1e-9;

  // lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void PCLLocalization::cloudReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "cloudReceived 1");
  if (!map_recieved_ /*|| !initialpose_recieved_*/)
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (use_imu_)
  {
    double received_time = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto &p : filtered_cloud_ptr->points)
  {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (scan_min_range_ < r && r < scan_max_range_)
    {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  // tf2::fromMsg(current_pose_stamped_.pose.pose, affine);
  Eigen::Matrix4f init_guess;
  Eigen::Affine3d affine;
  tf2::fromMsg(current_pose_stamped_.pose.pose, affine);
  init_guess = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();
  if (!registration_->hasConverged())
  {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  current_pose_stamped_.header.stamp = msg->header.stamp;
  current_pose_stamped_.pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  current_pose_stamped_.pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  current_pose_stamped_.pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  current_pose_stamped_.pose.pose.orientation = quat_msg;

  double pos_cov = registration_->getFitnessScore() / pose_position_cov_k; // The divisors here are arbitrary
  double ang_cov = registration_->getFitnessScore() / (5*pose_position_cov_k);
  current_pose_stamped_.pose.covariance = { pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, pos_cov, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, pos_cov, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, pos_cov, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, pos_cov, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, pos_cov};

  pose_pub_->publish(current_pose_stamped_);
  RCLCPP_INFO(get_logger(), "pub pose");

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = msg->header.stamp;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = odom_frame_id_;
  transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
  transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
  transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
  transform_stamped.transform.rotation = quat_msg;
  // broadcaster_.sendTransform(transform_stamped); // Comment this out to turn off transform publishing

  // path_.poses.push_back(current_pose_stamped_);
  // path_pub_->publish(path_);

  latest_fitness_score = registration_->getFitnessScore();

  if (enable_debug_)
  {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "[sec]" << std::endl;
    std::cout << "has converged: " << registration_->hasConverged() << std::endl;
    std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
                            (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
                       (final_transformation.coeff(0,
                                                   0) +
                        final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    std::cout << "TF EPSILON: " << registration_->getTransformationEpsilon() << std::endl;
    std::cout << "EUCL FIT EPSILON: " << registration_->getEuclideanFitnessEpsilon() << std::endl;
    std::cout << "TF ROT EPSILON: " << registration_->getTransformationRotationEpsilon() << std::endl;
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }

  if (enable_logging_to_file_)
  {
    *csvFile << current_pose_stamped_.pose.pose.position.x<<","
      <<current_pose_stamped_.pose.pose.position.y<<","<<current_pose_stamped_.pose.pose.position.z
      <<","<<registration_->getFitnessScore()<< std::endl;
  }

}
