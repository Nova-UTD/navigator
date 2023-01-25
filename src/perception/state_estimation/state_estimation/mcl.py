'''
Package: state_estimation
   File: mcl.py
 Author: Will Heitman (w at heit dot mn)

Python implementation of Monte Carlo Localization

MCL (a.k.a. particle filter-based localization) has five steps:
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

    def predict(self, particles: np.array, delta: np.array, std: np.array) -> None:
        """Move each particle based on a noisy motion prediction

        Args:
            particles (np.array): array of particles
            delta (np.array): [dx, dy, dtheta]
            std (np.array): standard deviation of delta
        """

        N = len(particles)
        # print(particles)
        # print(delta)
        particles[:] += delta  # + (randn(N) * std)
        particles += (randn(N, 3) * std)

        # Wrap heading to [0, 2*pi]
        particles[:, 2] %= 2 * np.pi

    def update_orig(self, particles: np.array, weights: np.array, z: np.array, R: np.array, landmarks: np.array) -> None:
        """Update the weights of each particle based on our current observation
        using Sequential Importance Sampling (SIS)

        Args:
            particles (np.array): particles to update
            weights (np.array): weights of our particles
            z (np.array): _description_
            R (np.array): sensor error standard deviation
            landmarks (np.array): landmark positions [x,y]
        """
        for i, landmark in enumerate(landmarks):
            distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
            weights *= scipy.stats.norm(distance, R).pdf(z[i])

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

    def update_weights(self, particles, weights, cloud: np.array, grid: np.array, gnss_pose, cloud_location=(100, 50)):
        """Update weights by checking each particle's alignment in the occupancy grid.

        Args:
            particles (_type_): _description_
            weights (_type_): _description_
            cloud (np.array): [[x,y], [x,y], ...]
            grid (np.array): n rows (x) from map_origin to origin+height_meters, m columns (y)
            from map_origin to origin+width_meters.
        """

        alignments = []

        for particle in particles:

            # We need the points in the particle frame.
            # They're in the base_link frame

            # First rotate:
            # Theta is difference in heading from base_link->particle
            theta = (particle[2] - self.mu[2]) % math.tau
            r = np.array([[np.cos(theta), -1*np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            # Apply the rotation matrix
            transformed_cloud = np.dot(cloud, r.T)

            # Then translate
            dx = particle[0] - self.mu[0]
            dy = particle[1] - self.mu[1]
            transformed_cloud[:] += [dx, dy]

            # Cloud should now be in the particle's frame

            # Scale to grid
            # This converts the point's x/y unit from "meters" to "cells"
            transformed_cloud /= self.grid_resolution

            # Translate so that (0,0) is at the grid's origin, not the particle's
            transformed_cloud += cloud_location

            # # Round each point in the cloud down to an int
            # # Now each point represents an index in grid. Convenient!
            grid_indices = transformed_cloud.astype(int)

            start = time.time()

            hits = 0

            for index in grid_indices[::100]:
                if index[0] >= grid.shape[0] or index[1] >= grid.shape[1]:
                    continue
                if grid[index[1], index[0]] == 100:
                    hits += 1

            alignments.append(hits)

        alignments = np.array(alignments)
        # plt.scatter(grid_indices[:,0], grid_indices[:,1])
        if (time.time() % 5 < 1):
            plt.hist(alignments)
            plt.show()
        # print(max(alignments))

        particles[:, 2] = gnss_pose[2]

        weights *= alignments
        dists = np.linalg.norm(particles[:, 0:2] - gnss_pose[0:2], axis=1)
        weights[dists > 4.0] *= 0.1  # Penalize particles too far from the GNSS

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

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

    def __init__(self, grid_resolution: float, initial_pose=np.array([0.0, 0.0, 0.0]), map_origin=np.array([0.0, 0.0]), N=100):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N
        self.mu = initial_pose

        self.map_origin = map_origin
        self.grid_resolution = grid_resolution

    def reset(self, initial_pose=np.array([0.0, 0.0, 0.0]), N=100):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N

    def step(self, delta: np.array, cloud: np.array, gnss_pose: np.array, grid: np.array) -> tuple:

        self.predict(self.particles, delta, std=np.array([0.0, 0.0, 0.0]))

        self.update_weights(self.particles, self.weights,
                            cloud, grid, gnss_pose, cloud_location=(100, 50))

        # Determine if a resample is necessary
        # N/2 is a good threshold
        N = len(self.particles)
        if self.neff(self.weights) < N/2:
            indexes = self.systematic_resample(self.weights)
            self.resample_from_index(self.particles, self.weights, indexes)
            assert np.allclose(self.weights, 1/N)
        mu, var = self.estimate(self.particles, self.weights)

        gnss_difference = np.linalg.norm(mu - gnss_pose)

        if gnss_difference > 5:
            self.reset(gnss_pose, N=100)

        self.mu = mu

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
