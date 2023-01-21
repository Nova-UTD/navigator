/*
 * Package:   mapping
 * Filename:  ParticleFilter.hpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#pragma once

// Message definitions
#include <carla_msgs/msg/carla_world_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numeric> // partial sum (cumulative sum)
#include <random>  // normal distributions
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
namespace navigator
{
  namespace perception
  {

    struct Pose
    {
      double x;
      double y;
      double h; // heading, in radians

      Pose(double x = 0.0, double y = 0.0, double h = 0.0) : x(x), y(y), h(h)
      {
      }

      Pose operator+(const Pose &a) const
      {
        Pose result(a.x + x, a.y + y, a.h + h);
        result.h = fmod(result.h, M_2_PI);
        return result;
      }
      Pose operator-(const Pose &a) const
      {
        Pose result(a.x - x, a.y - y, a.h - h);
        result.h = fmod(result.h, M_2_PI);
        return result;
      }
    };
    struct Particle : Pose
    {
      double w; // weight, the probability of a particle from [0.,1.]
      Particle(double x = 0.0, double y = 0.0, double h = 0.0, double w = 0.0) : Pose(x, y, h), w(w)
      {
      }
    };

    class ParticleFilter
    {
    public:
      ParticleFilter(int num_particles, const octomap::OcTree &tree) : tree(tree), num_particles(num_particles), is_initialized(false) {}
      virtual ~ParticleFilter();

      void init(double x, double y, double h, double std[]);

      void addMeasurement(Imu imu_msg);
      void predictMotion(Pose new_gnss_pose);
      void updateWeights();
      void resample();

      const bool isReady() const;

      PoseWithCovarianceStamped generatePose();

      PointCloud2 asPointCloud();
      PoseWithCovarianceStamped update(pcl::PointCloud<pcl::PointXYZI> observation, Pose gnss_pose);

    private:
      int num_particles;
      bool is_initialized;
      std::vector<double> weights;
      std::vector<Particle> particles;
      Pose gnss_pose_cached;
      pcl::PointCloud<pcl::PointXYZI> latest_observation;
      const octomap::OcTree &tree;

      double getParticleScore(const Particle p, pcl::PointCloud<pcl::PointXYZI> observation);
    };

  }
}
