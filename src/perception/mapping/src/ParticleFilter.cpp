/*
 * Package:   mapping
 * Filename:  ParticleFilter.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "mapping/ParticleFilter.hpp"

using namespace navigator::perception;
using namespace std::chrono_literals;

std::vector<Particle> ParticleFilter::generateParticles(Pose u, Pose stdev, int N)
{
  std::normal_distribution<> norm_x{u.x, stdev.x};
  std::normal_distribution<> norm_y{u.y, stdev.y};
  std::normal_distribution<> norm_h{u.h, stdev.h};
  std::vector<Particle> random_particles;

  double prob = 1.0 / N;

  for (int i = 0; i < N; i++)
  {
    Particle p;
    p.x = norm_x(gen);
    p.y = norm_y(gen);
    p.h = norm_h(gen);
    p.w = prob;
    random_particles.push_back(p);
  }

  return random_particles;
}

double quatToYaw(double w, double x, double y, double z)
{
  double t1 = 2.0 * (w * z + x * y);
  double t2 = 1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(t1, t2);

  return yaw;
}

ParticleFilter::ParticleFilter(PoseWithCovarianceStamped initial_guess, int N)
{
  this->N = N;

  auto q = initial_guess.pose.pose.orientation;
  double h = quatToYaw(q.w, q.x, q.y, q.z);

  Pose initial_guess_pose{
      initial_guess.pose.pose.position.x,
      initial_guess.pose.pose.position.y,
      h};

  Pose initial_guess_stdev{
      // Why these indices? See http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
      initial_guess.pose.covariance[0],
      initial_guess.pose.covariance[7],
      0.5 // radians
  };

  this->particles = generateParticles(
      initial_guess_pose, initial_guess_stdev, N);

  this->latest_time = initial_guess.header.stamp;
}

PointCloud2 ParticleFilter::asPointCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  for (Particle p : this->particles)
  {
    pcl::PointXYZRGB pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = 0.0;
    pt.r = 255;
    pt.g = 255;
    pt.b = 0;

    pcl_cloud.push_back(pt);
  }

  PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  ros_cloud.header.frame_id = "map";
  ros_cloud.header.stamp = this->latest_time;

  return ros_cloud;
}

ParticleFilter::~ParticleFilter()
{
}
