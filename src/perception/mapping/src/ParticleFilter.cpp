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

const bool ParticleFilter::isReady() const
{
  return is_initialized;
}

PoseWithCovarianceStamped ParticleFilter::update(PclCloud observation, Pose gnss_pose)
{
  std::printf("Updating...\n");

  // TODO: Make this more efficient. This copy is likely expensive.
  this->latest_observation = observation;

  auto start = std::chrono::high_resolution_clock::now();
  this->predictMotion(gnss_pose);
  auto stop = std::chrono::high_resolution_clock::now();
  int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
  std::printf("Motion update complete in %i ms\n", duration);

  start = std::chrono::high_resolution_clock::now();
  this->updateWeights();
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
  std::printf("Weight update complete in %i ms\n", duration);

  start = std::chrono::high_resolution_clock::now();
  this->resample();
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
  std::printf("Resample complete in %i ms\n", duration);

  gnss_pose_cached = gnss_pose;

  start = std::chrono::high_resolution_clock::now();
  PoseWithCovarianceStamped result = generatePose();
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
  std::printf("Weight update complete in %i ms\n", duration);

  printf("Result: (%f,%f,%f)/(%f,%f,%f,%f), N=%u\n",
         result.pose.pose.position.x,
         result.pose.pose.position.y,
         result.pose.pose.position.z,
         result.pose.pose.orientation.w,
         result.pose.pose.orientation.x,
         result.pose.pose.orientation.y,
         result.pose.pose.orientation.z,
         this->particles.size());
  return result;
}

/**
 * @brief Add predicted displacement, plus noise, to each particle
 *
 * @param new_gnss_pose The latest (x,y,h) from the GNSS.
 */
void ParticleFilter::predictMotion(Pose new_gnss_pose)
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

  std::default_random_engine rand_gen;

  auto p = this->particles.begin();
  while (p != this->particles.end())
  {
    p->x += (displacement.x + dist_x(rand_gen));
    p->y += (displacement.y + dist_y(rand_gen));
    p->h += (displacement.h + dist_h(rand_gen));
    p->h = fmod(displacement.h, M_2_PI); // Wrap to [0, 2*pi]
    p++;
  }

  // std::printf("Is now: (%f,%f,%f)\n",
  //             this->particles.front().x,
  //             this->particles.front().y,
  //             this->particles.front().h);

  gnss_pose_cached = new_gnss_pose;
}

double ParticleFilter::getParticleScore(const Particle p, PclCloud observation)
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
  baselink_to_map_tf(0, 3) = p.x;
  baselink_to_map_tf(1, 3) = p.y;
  baselink_to_map_tf(3, 3) = 1.0;

  // Use the Eigen transform to transform the pcl cloud to the global "map" frame
  pcl::transformPointCloud(observation, observation, baselink_to_map_tf);

  for (pcl::PointXYZI pt : observation)
  {
    octomap::OcTreeNode *node = this->tree.search(pt.x, pt.y, pt.z, 15); // Depth of 15 = 0.4m
    if (node == nullptr)
      continue;                             // No points awarded if node not yet added to octree
    particle_score += node->getOccupancy(); // [0.0, 1.0]
  }
  return particle_score;
}

/**
 * @brief Perform Sequential Importance Sampling
 *
 */
void ParticleFilter::updateWeights()
{
  double total_score = 0.0; // Used to normalize the probability

  // Loop through each particle
  auto p = this->particles.begin();
  while (p != this->particles.end())
  {
    double score = getParticleScore(*p, latest_observation);
    p->w *= score; // Bayes theorem: P(x|z) = (likelihood * prior) / normalization
    total_score += score;
    p++;
  }

  // Normalize such that the sum of all weights = 1.0
  for (int i = 0; i < particles.size(); i++)
  {
    particles.at(i).w /= total_score;
    weights.at(i) = particles.at(i).w;
  }
}

void ParticleFilter::resample()
{
  std::default_random_engine rand_eng;

  // A distribution from (0, weights.size()) where the probability
  // of each element being chosen is based on each weight, such that
  // higher weights are more likely to be selected.
  std::discrete_distribution<int> weighted_dist(weights.begin(), weights.end());

  std::vector<Particle> resampled_particles;

  const double GNSS_DISTANCE_THRESHOLD = 3.0; // Particles further than this distance will be removed.

  for (int i = 0; i < num_particles; i++)
  {
    Particle random_particle = particles[weighted_dist(rand_eng)];

    double distance_from_gnss = sqrt(
        pow(gnss_pose_cached.x - random_particle.x, 2) +
        pow(gnss_pose_cached.y - random_particle.y, 2));

    if (distance_from_gnss > GNSS_DISTANCE_THRESHOLD)
    {
      resampled_particles.push_back(Particle(
          gnss_pose_cached.x,
          gnss_pose_cached.y,
          gnss_pose_cached.h,
          random_particle.w));
    }
    else
    {
      resampled_particles.push_back(random_particle);
    }
  }

  particles = resampled_particles;
}

/**
 * @brief
 *
 * @param x Initial x position from GNSS (m)
 * @param y Initial y position from GNSS (m)
 * @param heading Initial heading from GNSS (m)
 * @param std [stdev_x, stdev_y, stdev_heading] (m)
 */
void ParticleFilter::init(double x, double y, double heading, double std[])
{
  weights.resize(num_particles);
  particles.resize(num_particles);

  gnss_pose_cached = Pose(x, y, heading);

  // Standard deviations for x, y, and theta
  double std_x, std_y, std_heading;
  std_x = std[0];
  std_y = std[1];
  std_heading = std[2];

  // Normal distributions
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(heading, std_heading);

  std::default_random_engine rand_gen;

  // create particles and set their values
  for (int i = 0; i < num_particles; ++i)
  {
    Particle p;
    p.x = dist_x(rand_gen); // take a random value from the Gaussian Normal distribution and update the attribute
    p.y = dist_y(rand_gen);
    p.h = dist_theta(rand_gen);
    p.w = 1;

    particles[i] = p;
    weights[i] = p.w;
  }
  is_initialized = true;
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

  // Construct quaternion from roll=0, pitch=0, yaw
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(mean_pose.h, Eigen::Vector3f::UnitZ());

  auto pose_msg = PoseWithCovarianceStamped();
  pose_msg.pose.pose.position.x = mean_pose.x;
  pose_msg.pose.pose.position.y = mean_pose.y;
  pose_msg.pose.pose.orientation.w = q.w();
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();

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

  return ros_cloud;
}

ParticleFilter::~ParticleFilter()
{
}
