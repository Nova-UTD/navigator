/*
 * Package:   mapping
 * Filename:  ParticleFilter.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 *
 * General steps to particle filters:
 * 1. Generate a random set of particles in a normal distribution around an initial guess
 * 2. Given odometry, predict the motion of all particles and update their poses
 * 3. Given observations, assign a weight (probability) to each particle
 * 4. Resample the particles, removing or copying each, based on their weights
 * 5. Calculate the pose using the mean and stddev of the particles
 * 6. Repeat 2-5 in a loop.
 */

#include "mapping/ParticleFilter.hpp"

#include <iostream>

using namespace navigator::perception;
using namespace std::chrono_literals;
typedef pcl::PointCloud<pcl::PointXYZI> PclCloud;

PoseWithCovarianceStamped ParticleFilter::update(PclCloud observation, Pose gnss_pose)
{
  std::printf("Updating...\n");

  // TODO: Make this more efficient. This copy is likely expensive.
  this->latest_observation = std::make_shared<PclCloud>(PclCloud(observation));

  this->predictParticleMotion(gnss_pose);
  std::printf("Motion update complete.\n");
  this->updateParticleWeights();
  std::printf("Weight update complete.\n");
  this->resample();
  std::printf("Resample complete.\n");

  gnss_pose_cached = gnss_pose;
  std::printf("Update COMPLETE!\n");
  return this->generatePose();
}

/**
 * @brief Add predicted displacement, plus noise, to each particle
 *
 * @param new_gnss_pose The latest (x,y,h) from the GNSS.
 */
void ParticleFilter::predictParticleMotion(Pose new_gnss_pose)
{
  Pose displacement = new_gnss_pose - gnss_pose_cached;

  // std::printf("Displacement: (%f,%f,%f)\n",
  //             displacement.x,
  //             displacement.y,
  //             displacement.h);

  std::normal_distribution<> dist_x{displacement.x, 4.0}; // 4m error -- actual GNSS error is ~2m
  std::normal_distribution<> dist_y{displacement.y, 4.0};
  std::normal_distribution<> dist_h{displacement.h, 0.35}; // ~20 degrees error

  // Add predicted displacement, plus noise, to each particle
  // std::printf("Was:    (%f,%f,%f)\n",
  //             this->particles.front().x,
  //             this->particles.front().y,
  //             this->particles.front().h);

  auto p = this->particles.begin();
  while (p != this->particles.end())
  {
    p->x += (displacement.x + dist_x(gen));
    p->y += (displacement.y + dist_y(gen));
    p->h += (displacement.h + dist_h(gen));
    p->h = fmod(displacement.h, M_2_PI); // Wrap to [0, 2*pi]
    p++;
  }

  // std::printf("Is now: (%f,%f,%f)\n",
  //             this->particles.front().x,
  //             this->particles.front().y,
  //             this->particles.front().h);
}

double ParticleFilter::getAlignmentRatio(const Particle p, PclCloud observation)
{
  double particle_score = 0.0;

  // 1. Transform observation from sensor frame to map frame

  Eigen::Matrix4f baselink_to_map_tf = Eigen::Matrix4f::Zero();
  /**
   * https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
   *
   * cos(h) -sin(h) 0   x
   * sin(h)  cos(h) 0   y
   * 0       0      1   z
   * 0       0      0   1
   */
  baselink_to_map_tf(0, 0) = cos(p.h);
  baselink_to_map_tf(0, 1) = -1 * sin(p.h);
  baselink_to_map_tf(1, 0) = sin(p.h);
  baselink_to_map_tf(1, 1) = cos(p.h);
  baselink_to_map_tf(2, 2) = 1.0;
  baselink_to_map_tf(0, 3) = cos(p.x);
  baselink_to_map_tf(1, 3) = cos(p.y);
  baselink_to_map_tf(3, 3) = 1.0;

  // Use the Eigen transform to transform the pcl cloud to the global "map" frame
  pcl::transformPointCloud(observation, observation, baselink_to_map_tf);

  for (pcl::PointXYZI pt : observation)
  {
    octomap::OcTreeNode *node = this->tree.search(pt.x, pt.y, pt.z);
    if (node == nullptr)
      continue;                             // No points awarded if node not yet added to octree
    particle_score += node->getOccupancy(); // [0.0, 1.0]
  }
  return particle_score;
}

void ParticleFilter::updateParticleWeights()
{
  double total_score = 0.0; // Used to normalize the probability

  // Loop through each particle
  auto p = this->particles.begin();
  while (p != this->particles.end())
  {
    double score = getAlignmentRatio(*p, PclCloud(*this->latest_observation));
    p->w = score;
    total_score += score;
    p++;
  }

  // Normalize such that the sum of all weights = 1.0
  p = this->particles.begin();
  while (p != this->particles.end())
  {
    p->w /= total_score;
    p++;
  }
}

void ParticleFilter::resample()
{
  Particle most_likely_particle;
  auto p = this->particles.begin();
  while (p != this->particles.end())
  {
    if (p->w > most_likely_particle.w)
    {
      most_likely_particle = *p;
    }
    p++;
  }
  std::printf("Highest score: (%f, %f, %f) with %f",
              most_likely_particle.x,
              most_likely_particle.y,
              most_likely_particle.h,
              most_likely_particle.w);
}

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
    std::cout << p.x << ", " << p.y << ", " << p.h << std::endl;
  }

  return random_particles;
}

double quaternionToYaw(double w, double x, double y, double z)
{
  double t1 = 2.0 * (w * z + x * y);
  double t2 = 1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(t1, t2);

  return yaw;
}

ParticleFilter::ParticleFilter(PoseWithCovarianceStamped initial_guess, int N, const octomap::OcTree &tree) : tree(tree), N(N)
{
  std::cout << "Tree has " << this->tree.size() << " nodes\n";

  auto q = initial_guess.pose.pose.orientation;
  double h = quaternionToYaw(q.w, q.x, q.y, q.z);

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

PoseWithCovarianceStamped ParticleFilter::generatePose()
{
  Pose mean_pose;
  Pose variance_pose;

  // 1. Calculate mean
  for (Particle p : this->particles)
  {
    mean_pose.x += p.x;
    mean_pose.y += p.y;
    mean_pose.h += p.h;
  }

  mean_pose.x /= this->particles.size();
  mean_pose.y /= this->particles.size();
  mean_pose.h /= this->particles.size();
  mean_pose.h = fmod(mean_pose.h, M_2_PI); // Wrap to [0, 2*pi]

  // 2. Calculate variance
  // https://en.wikipedia.org/wiki/Variance#Discrete_random_variable
  for (Particle p : this->particles)
  {
    variance_pose.x += pow(p.x - mean_pose.x, 2);
    variance_pose.y += pow(p.y - mean_pose.y, 2);
    variance_pose.h += pow(p.h - mean_pose.h, 2);
  }
  variance_pose.x /= this->particles.size();
  variance_pose.y /= this->particles.size();
  variance_pose.h /= this->particles.size();
  variance_pose.h = fmod(variance_pose.h, M_2_PI); // Wrap to [0, 2*pi]

  auto pose_msg = PoseWithCovarianceStamped();
  pose_msg.pose.pose.position.x = mean_pose.x;
  pose_msg.pose.pose.position.y = mean_pose.y;
  pose_msg.pose.pose.orientation.w = cos(mean_pose.h / 2);
  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = sin(mean_pose.h / 2);

  // std::printf("Pose is now: (%f,%f,%f)\n",
  //             mean_pose.x,
  //             mean_pose.y,
  //             mean_pose.h);

  return pose_msg;
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
    pt.r = 100;
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