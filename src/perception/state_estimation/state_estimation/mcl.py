'''
Package: state_estimation
   File: mcl.py
 Author: Will Heitman (w at heit dot mn)

Python implementation of Monte Carlo Localization

MCL (a.k.a. a particle filter) has five steps:
1. Generate possible poses (particles) in a Gaussian distribution around an initial guess.
2. Predict the next state of the particles with a motion_update, which includes noise
3. For each updated particle, assign a probability score using the latest observations
4. Either remove or copy a particle randomly, with bias from the particle's score
5. Repeat 2-5.

The state estimate is the mean and covariance of the particles.
The particles (poses) gradually converge.

A particle is an np.array: [x, y, heading, weight]

ACKNOWLEDGEMENT:
This code is adapted from Roger Labbe's work:
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb

For a detailed explanation of how the filter works, visit the above link.

'''

from numpy.random import seed
import math
import numpy as np
from numpy.random import randn, random
import scipy
import scipy.stats
import time
import matplotlib.pyplot as plt

ROAD_ID = 4286595200
TRAFFIC_LIGHT_ID = 4294617630
POLE_ID = 4288256409

GRID_ORIGIN_METERS = [-20., -30.]
GRID_ORIGIN_METERS = np.array(GRID_ORIGIN_METERS)
CELL_SIZE = 0.4  # meters/cell

DO_PLOT_ALIGNMENT = False


class MCL:

    def create_gaussian_particles(self, mean: np.array, std: tuple, N: int) -> np.array:
        """Return particles in a Gaussian distribution

        Args:
            mean (np.array): [x, y, theta]
            std (tuple): [std_x, std_y, std_theta]
            N (int): Number of particles to generate

        Returns:
            np.array: Particles in the new distribution
        """
        particles: np.array = np.empty((N, 3))
        particles[:, 0] = mean[0] + (randn(N) * std[0])
        particles[:, 1] = mean[1] + (randn(N) * std[1])
        particles[:, 2] = mean[2] + (randn(N) * std[2])
        particles[:, 2] %= 2 * np.pi

        return particles

    def predictMotion(self, particles, u, std, dt, new_heading=None):
        """ move according to control input u (heading rate, speed)
        with noise Q (std heading change, std velocity)`"""

        N = len(particles)
        # update heading

        if new_heading is None:
            particles[:, 2] += u[0] * dt + (randn(N) * std[0])
            particles[:, 2] %= 2 * np.pi
        else:
            particles[:, 2] = new_heading

        # move in the (noisy) commanded direction
        dist = (self.previous_speed * dt) + (randn(N) * std[1])
        particles[:, 0] += np.cos(particles[:, 2]) * dist
        particles[:, 1] += np.sin(particles[:, 2]) * dist

        self.previous_speed = u[1]

    def updateOriginal(self, particles: np.array, weights: np.array, z: np.array, R: np.array, landmarks: np.array) -> None:
        """Update the weights of each particle based on our current observation
        using Sequential Importance Sampling (SIS)

        Args:
            particles (np.array): particles to update
            weights (np.array): weights of our particles
            z (np.array): Distances to nearby landmarks
            R (np.array): sensor error standard deviation
            landmarks (np.array): landmark positions [x,y]
        """

        if len(landmarks) < 1:
            return
        if len(z) < 1:
            return

        # Consider only the nearest landmark
        landmark = landmarks[np.argmin(
            np.linalg.norm(landmarks-self.mu[:2], axis=1))]
        print(f"Nearest lm: {landmark}")

        particle_distances = np.linalg.norm(particles[:, :2]-landmark, axis=1)
        print(f"Particle dists: {particle_distances}")
        print(f"Observed dists: {z}")

        # pdf() gives the likelihood that the observed distance z
        # matches the particle's distance. Higher likelihoods
        # mean relatively higher weights.
        weights *= scipy.stats.norm(particle_distances, R).pdf(z[0])

        plt.scatter(particles[:, 0], particles[:, 1], c=weights)
        plt.show()

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

    def updateWeights(self, particles, weights, cloud: np.array, grid: np.array, gnss_pose):
        """Update the weights of each particle based on our current observation
        using Sequential Importance Sampling (SIS)

        Args:
            particles (np.array): particles to update
            weights (np.array): weights of our particles
            z (np.array): _description_
            R (np.array): sensor error standard deviation
            landmarks (np.array): landmark positions [x,y]
        """

        # Crop cloud to nearby
        nearby_cloud = cloud[np.linalg.norm(cloud[:, 0:2], axis=1) < 14]

        # Transform cloud to grid
        # 1. Translate to grid origin
        cloud_on_grid = np.copy(nearby_cloud)

        cloud_on_grid[:, 0:2] -= GRID_ORIGIN_METERS
        # 2. Scale to cell size
        cloud_on_grid[:, 0:2] /= CELL_SIZE

        alignments = []

        plt.imshow(grid, origin='lower')
        # plt.scatter(cloud_on_grid[:, 0], cloud_on_grid[:, 1], c='red', s=1.0)

        particles_on_grid = []

        for particle in particles:
            # particle_on_grid = np.copy(particle)

            # Transform particle to grid s.t. x/y are grid indices
            particle_on_grid = particle - self.mu
            particle_on_grid[0:2] -= GRID_ORIGIN_METERS
            particle_on_grid[0:2] /= CELL_SIZE
            particle_on_grid = particle_on_grid.astype(int)

            # Rotate cloud to particle frame

            cloud_on_particle = np.copy(cloud_on_grid)
            cloud_on_particle[:, 0:2] += GRID_ORIGIN_METERS/CELL_SIZE
            d_theta = particle_on_grid[2]
            r = np.array([[np.cos(d_theta), -1*np.sin(d_theta)],
                          [np.sin(d_theta), np.cos(d_theta)]])
            cloud_on_particle[:, 0:2] = np.dot(
                cloud_on_particle[:, 0:2], r.T)  # Apply rotation matrix
            # cloud_on_grid[:, 0:2] -= GRID_ORIGIN_METERS/CELL_SIZE

            # Translate cloud to complete transform
            cloud_on_particle[:, 0:2] += particle_on_grid[0:2]
            cloud_on_particle = cloud_on_particle.astype(int)

            hits = 0
            good_points = []
            bad_points = []
            for pt in cloud_on_particle:
                if pt[0] >= grid.shape[0] or pt[1] >= grid.shape[1] or pt[0] < 0 or pt[1] < 0:
                    continue
                if grid[pt[1]][pt[0]] == 100 and pt[2] == ROAD_ID:
                    hits += 1
                    good_points.append(pt)
                elif grid[pt[1]][pt[0]] == 100 and (pt[2] == POLE_ID or pt[2] == TRAFFIC_LIGHT_ID):
                    # Pole and traffic light misalignment penalty
                    # Poles and traffic lights should not fall into roads, so penalize particles that present this
                    hits -= 5
                else:
                    bad_points.append(pt)
            alignments.append(hits / len(cloud_on_particle))

            bad_points = np.array(bad_points)
            good_points = np.array(good_points)
            # plt.scatter(particle_on_grid[0], particle_on_grid[1], c='blue')
            # # # print(cloud_on_particle)

            particles_on_grid.append(particle_on_grid)

            if not DO_PLOT_ALIGNMENT:
                continue  # Skip to next loop if plotting not enabled

            if len(bad_points) > 1:
                plt.scatter(bad_points[:, 0],
                            bad_points[:, 1], s=1.0, c='red')
            if len(good_points) > 1:
                plt.scatter(good_points[:, 0],
                            good_points[:, 1], s=1.0, c='green')

        particles_on_grid = np.array(particles_on_grid)

        if DO_PLOT_ALIGNMENT:
            print(np.max(alignments) / len(cloud))
            plt.scatter(particles_on_grid[:, 0],
                        particles_on_grid[:, 1], c=alignments)

            plt.colorbar()
            plt.show()

        # alignments = np.array(alignments) / len(particles)

        # # if np.max(alignments) > 0.5:
        # #     weights *= alignments
        # # else:
        # #     print(f"Max alignment was only {np.max(alignments)}")
        # weights *= alignments

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

        return alignments

    def estimate(self, particles: np.array, weights) -> tuple:
        """Return mean and variance of particles

        Args:
            particles (np.array): _description_
            weights (_type_): _description_

        Returns:
            tuple: _description_
        """

        mean = np.average(particles, weights=weights, axis=0)
        var = np.average((particles - mean)**2, weights=weights, axis=0)
        return mean, var

    def simple_resample(particles: np.array, weights: np.array) -> None:
        N = len(particles)
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.  # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, random(N))

        # resample according to indexes
        particles[:] = particles[indexes]
        weights.fill(1.0 / N)

    def neff(self, weights: np.array) -> float:
        return 1. / np.sum(np.square(weights))

    def resample_from_index(self, particles: np.array, weights, indexes) -> None:
        particles[:] = particles[indexes]
        weights.resize(len(particles))
        weights.fill(1.0 / len(weights))

    def create_uniform_particles(self, x_range, y_range, hdg_range, N: int) -> np.array:
        particles: np.array = np.empty((N, 3))
        particles[:, 0] = np.random.uniform(x_range[0], x_range[1], size=N)
        particles[:, 1] = np.random.uniform(y_range[0], y_range[1], size=N)
        particles[:, 2] = np.random.uniform(hdg_range[0], hdg_range[1], size=N)
        particles[:, 2] %= 2 * np.pi
        return particles

    def __init__(self, clock, grid_resolution: float, initial_pose=np.array([0.0, 0.0, 0.0]), map_origin=np.array([0.0, 0.0]), N=30):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N
        self.alignment_history = None
        self.mu = initial_pose

        self.map_origin = map_origin
        self.grid_resolution = grid_resolution

        self.mus = []
        self.gnss_poses = []
        self.last_update_time = clock
        self.previous_speed = 0.0

    def reset(self, initial_pose, N):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N

    def addNoise(self, particles, std, N):
        threshold = 1/N

        low_particles = particles[self.weights < threshold]
        low_N = len(low_particles)

        print(f"Adding noise to {low_N} particles")
        particles[self.weights < threshold][:, 0] += (randn(low_N) * std[0])
        particles[self.weights < threshold][:, 1] += (randn(low_N) * std[1])
        particles[self.weights < threshold][:, 2] += (randn(low_N) * std[2])
        particles[self.weights < threshold][:, 2] %= 2 * np.pi

    def sense(self, cloud, noise):

        MAX_LANDMARK_DIST = 10.0  # meters

        traffic_light_pts = cloud[cloud[:, 2] == TRAFFIC_LIGHT_ID][:, 0:2]

        # Filter out faraway pts
        traffic_light_pts = traffic_light_pts[np.linalg.norm(
            traffic_light_pts, axis=1) < MAX_LANDMARK_DIST]

        downsampled_pts = []

        # Only include points not super close to other ones
        MIN_DISTANCE = 1.0  # Points closer than this will be simplified to one pt
        for pt_a in traffic_light_pts:
            too_close = False
            for pt_b in downsampled_pts:
                if np.linalg.norm(pt_a - pt_b) < MIN_DISTANCE:
                    too_close = True
                    break
            if not too_close:
                downsampled_pts.append(pt_a)

        downsampled_pts = np.array(downsampled_pts)

        print("Traffic light points:")
        print(downsampled_pts)

        downsampled_pts = downsampled_pts.reshape((-1, 2))

        distances = np.linalg.norm(
            downsampled_pts, axis=1) + randn(downsampled_pts.shape[0]) * noise

        return distances

    def getLandmarks(self, grid, mu):
        # plt.imshow(grid, origin='lower')
        # plt.show()
        landmarks_on_grid = np.flip(
            np.transpose((grid == 13).nonzero()), axis=1)

        if landmarks_on_grid.shape[0] == 0:
            return landmarks_on_grid  # If empty, send it out

        print(landmarks_on_grid)
        # landmarks_on_grid = landmarks_on_grid[landmarks_on_grid[:, 0] > 70]
        # landmarks_on_grid = landmarks_on_grid[landmarks_on_grid[:, 0] < 85]
        # landmarks_on_grid = landmarks_on_grid[landmarks_on_grid[:, 1] < 70]
        # landmarks_on_grid = landmarks_on_grid[landmarks_on_grid[:, 1] > 60]

        landmarks_on_map = landmarks_on_grid * CELL_SIZE
        landmarks_on_map += GRID_ORIGIN_METERS

        # Rotate to map
        d_theta = -1 * mu[2]  # Invert to get base_link->map rotation
        r = np.array([[np.cos(d_theta), -1*np.sin(d_theta)],
                      [np.sin(d_theta), np.cos(d_theta)]])
        landmarks_on_map[:, 0:2] = np.dot(
            landmarks_on_map[:, 0:2], r.T)  # Apply rotation matrix

        # Now translate
        landmarks_on_map += mu[0:2]

        print(landmarks_on_map)

        # plt.scatter(landmarks_on_map[:, 0], landmarks_on_map[:, 1])
        # plt.scatter(mu[0], mu[1], c='r')
        # plt.show()

        return landmarks_on_map

    def resample(self, particles, alignments, gnss_pose):

        alignments = np.array(alignments).reshape((-1, 1))

        # Kill off particles further than 5m from GNSS
        dists = gnss_difference = np.linalg.norm(
            particles[:, 0:2] - gnss_pose[0:2], axis=1)
        alignments[dists > 5] = 0.

        # Form a combined particle-alignment array
        combined_array = np.hstack((particles, alignments))
        # Sort by third column (alignments)
        combined_array = combined_array[combined_array[:, 3].argsort()]

        # Determine the number of particles to be replaced
        replacement_qty = int(len(particles)/10)

        # Perform replacement
        print(f"Replacing bottom {replacement_qty} particles")
        combined_array[0:replacement_qty] = combined_array[-
                                                           1*replacement_qty-1:-1]

        # Add a little noise
        print(randn(replacement_qty).T)
        print(combined_array[0: replacement_qty, 0])
        combined_array[0: replacement_qty,
                       0] += randn(replacement_qty).T * 0.1
        combined_array[0: replacement_qty,
                       1] += randn(replacement_qty).T * 0.1

        print(combined_array)

        # Return only the particles, not the alignments
        return combined_array[:, 0: 3]

    def step(self, u, clock, cloud: np.array, gnss_pose: np.array, grid: np.array) -> tuple:
        """Takes new data, runs it through the filter, and generates a result pose.

        ✅ 1. Predict the motion of all particles using the latest speedometer and angular velocity data.
        ✅ 2. Assign a likelihood score to each particle using the latest classified cloud and grid.
        ❌ 3. Select and replace the bottom X percent of particles with copies of the top X percent
        ✅ 4. Take the weighted mean and covariance of the particles, return them
        ✅ 5. Repeat 1-4.

        Returns:
            (mean, covariance)
        """

        self.predictMotion(self.particles, u, std=[
                           0.3, 0.05], dt=clock - self.last_update_time, new_heading=gnss_pose[2])
        self.last_update_time = clock

        alignments = self.updateWeights(self.particles, self.weights,
                                        cloud, grid, gnss_pose)

        self.particles = self.resample(self.particles, alignments, gnss_pose)

        mu, var = self.estimate(self.particles, self.weights)

        gnss_difference = np.linalg.norm(mu - gnss_pose)
        if gnss_difference > 3:
            print("KIDNAPPED! Reseting.")
            self.reset(gnss_pose, N=len(self.particles))

        self.mu = mu

        self.mus.append(mu)
        self.gnss_poses.append(gnss_pose)

        # plt.plot(self.mus)
        # plt.plot(self.gnss_poses)
        # plt.show()

        return mu, var

    # From Roger Labbe's filterpy
    # https://github.com/rlabbe/filterpy/blob/master/filterpy/monte_carlo/resampling.py
    def systematic_resample(self, weights: np.array) -> np.array:
        """ Performs the systemic resampling algorithm used by particle filters.
        This algorithm separates the sample space into N divisions. A single random
        offset is used to to choose where to sample from for all divisions. This
        guarantees that every sample is exactly 1/N apart.
        Parameters
        ----------
        weights : list-like of float
            list of weights as floats
        Returns
        -------
        indexes : ndarray of ints
            array of indexes into the weights defining the resample. i.e. the
            index of the zeroth resample is indexes[0], etc.
        """
        N = len(weights)

        # make N subdivisions, and choose positions with a consistent random offset
        positions = (random() + np.arange(N)) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes
