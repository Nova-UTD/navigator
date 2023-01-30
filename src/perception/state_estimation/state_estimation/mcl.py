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

    def predict(self, particles, u, std, dt=1.):
        """ move according to control input u (heading rate, speed)
        with noise Q (std heading change, std velocity)`"""

        N = len(particles)
        # update heading
        particles[:, 2] += u[0] * dt + (randn(N) * std[0])
        particles[:, 2] %= 2 * np.pi

        # move in the (noisy) commanded direction
        dist = (u[1] * dt) + (randn(N) * std[1])
        particles[:, 0] += np.cos(particles[:, 2]) * dist
        particles[:, 1] += np.sin(particles[:, 2]) * dist

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

    def update_weights(self, particles, weights, cloud: np.array, grid: np.array, gnss_pose):
        """Update weights by checking each particle's alignment in the occupancy grid.

        Args:
            particles (_type_): _description_
            weights (_type_): _description_
            cloud (np.array): [[x,y], [x,y], ...]
            grid (np.array): n rows (x) from map_origin to origin+height_meters, m columns (y)
            from map_origin to origin+width_meters.
        """

        if self.alignment_history is None:
            self.alignment_history = np.array([])

        alignments = []

        max_idx = np.argmax(weights)
        min_idx = np.argmin(weights)

        for idx, particle in enumerate(particles):

            # Our LiDAR data is in the vehicle ("base_link") frame.
            # In order for us to test alignment, we need this in the particle's frame

            # Rotate from base_link to particle frame
            if self.mu is None:
                translation = np.zeros((3))  # Empty
            else:
                translation = particle - self.mu
            d_theta = translation[2]

            r = np.array([[np.cos(d_theta), -1*np.sin(d_theta)],
                          [np.sin(d_theta), np.cos(d_theta)]])
            transformed_cloud = np.dot(cloud, r.T)  # Apply rotation matrix

            # Then translate
            transformed_cloud[:] += translation[0:2]

            # We're now in the particle frame

            # Scale to grid
            transformed_cloud /= self.grid_resolution
            transformed_cloud += [50., 75.]  # TODO: Make this a param

            # # Round each point in the cloud down to an int
            # # Now each point represents an index in grid. Convenient!
            grid_indices = transformed_cloud.astype(int)

            hits = 0

            for index in grid_indices:
                if index[0] >= grid.shape[0] or index[1] >= grid.shape[1]:
                    continue
                if grid[index[1], index[0]] == 100:
                    hits += 1
                else:
                    hits -= 1

            # if idx == max_idx:
            #     print(f"Best hits: {(hits/len(grid_indices))*100}%")
            #     plt.imshow(grid, origin='lower')
            #     plt.scatter(
            #         grid_indices[:, 0], grid_indices[:, 1])
            #     plt.title(f"Best hits: {(hits/len(grid_indices))*100}%")
            #     plt.show()

            # elif idx == min_idx:
            #     print(f"Worst hits: {(hits/len(grid_indices))*100}")
            #     plt.imshow(grid, origin='lower')
            #     plt.scatter(
            #         grid_indices[:, 0], grid_indices[:, 1])
            #     plt.title(f"Worst hits: {(hits/len(grid_indices))*100}")
            #     plt.show()

            alignments.append(hits)

        # Plot for debugging
        # Plot particle with best alignment
        if self.mu is not None and time.time() % 5 < 1:
            # plt.imshow(grid, origin='lower')
            u = np.cos(particles[:, 2])
            v = np.sin(particles[:, 2])
            particles_on_grid = particles - self.mu
            particles_on_grid[:, 0:2] += [50., 75.]
            # plt.xlim((20, 80))
            # plt.ylim((60, 100))
            # plt.scatter(particles_on_grid[:, 0], particles_on_grid[:, 1],c=self.weights)
            # plt.show()

        alignments = np.array(alignments)
        self.alignment_history = np.append(
            self.alignment_history, np.mean(alignments))
        # print(self.alignment_history)

        # if len(self.alignment_history) % 50 == 0:
        #     plt.plot(range(len(self.alignment_history)), self.alignment_history)
        #     plt.show()

        particles[:, 2] = gnss_pose[2]

        weights *= alignments
        # dists = np.linalg.norm(particles[:, 0:2] - gnss_pose[0:2], axis=1)
        # weights[dists > 4.0] *= 0.1  # Penalize particles too far from the GNSS

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

    def get_best_particle(self, particles, cloud: np.array, grid: np.array):

        GRID_ORIGIN_METERS = [-20., -30.]
        CELL_SIZE = 0.4  # meters/cell

        # Crop cloud to nearby
        nearby_cloud = cloud[np.linalg.norm(cloud, axis=1) < 14]

        # Transform cloud to grid
        # 1. Translate to grid origin
        cloud_on_grid = nearby_cloud - GRID_ORIGIN_METERS
        # 2. Scale to cell size
        cloud_on_grid /= CELL_SIZE
        # 3. Crop to nearby

        best_alignment = 0
        best_particle = np.zeros((3))
        best_good_pts = np.zeros((3))
        best_bad_pts = np.zeros((3))

        for particle in particles:
            cloud_in_particle_frame = cloud_on_grid.copy()
            alignment = 0
            good_pts = []
            bad_pts = []

            # Rotate
            r = np.array([[np.cos(particle[2]), -1*np.sin(particle[2])],
                          [np.sin(particle[2]), np.cos(particle[2])]])
            cloud_in_particle_frame = np.dot(
                cloud_in_particle_frame, r.T)  # Apply rotation matrix

            # Translate
            cell_translation = particle[0:2] / CELL_SIZE
            cloud_in_particle_frame += cell_translation

            # Round to ints
            cloud_in_particle_frame = cloud_in_particle_frame.astype(int)

            # plt.scatter(
            #     cloud_in_particle_frame[:, 0], cloud_in_particle_frame[:, 1], s=0.1)

            for pt in cloud_in_particle_frame:
                if pt[0] >= grid.shape[0] or pt[1] >= grid.shape[1]:
                    # print("Skipping")
                    continue
                is_aligned = grid[pt[1], pt[0]] != 0
                if is_aligned:
                    alignment += 1
                    good_pts.append(pt)
                else:
                    bad_pts.append(pt)

            if alignment > best_alignment:
                best_alignment = alignment
                best_particle = particle
                best_good_pts = np.array(good_pts)
                best_bad_pts = np.array(bad_pts)
                best_cloud = cloud_in_particle_frame

        # plt.imshow(grid, origin='lower')
        # best_particle_on_grid = np.copy(best_particle)

        # best_particle_on_grid[0:2] -= GRID_ORIGIN_METERS
        # best_particle_on_grid[0:2] /= CELL_SIZE

        # plt.scatter(best_particle_on_grid[0],
        #             best_particle_on_grid[1], c='blue', s=15.0)

        # plt.scatter(50, 75, c='red', s=15.0)
        # plt.scatter(best_bad_pts[:, 0], best_bad_pts[:, 1], c='red', s=0.5)
        # plt.scatter(
        #     best_good_pts[:, 0], best_good_pts[:, 1], c='green', s=0.5)
        # print(f"Best alignment: {best_alignment} @ {best_particle}")
        # plt.show()

        return best_particle, [0., 0., 0.], alignment

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

    def __init__(self, grid_resolution: float, initial_pose=np.array([0.0, 0.0, 0.0]), map_origin=np.array([0.0, 0.0]), N=30):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N
        self.alignment_history = None
        self.mu = None

        self.map_origin = map_origin
        self.grid_resolution = grid_resolution

        self.mus = []

    def reset(self, initial_pose=np.array([0.0, 0.0, 0.0]), N=30):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(2, 2, np.pi/8), N=N)

        self.weights = np.ones(N) / N

    def add_noise(self, particles, std, N):
        threshold = 1/N

        low_particles = particles[self.weights < threshold]
        low_N = len(low_particles)

        print(f"Adding noise to {low_N} particles")
        particles[self.weights < threshold][:, 0] += (randn(low_N) * std[0])
        particles[self.weights < threshold][:, 1] += (randn(low_N) * std[1])
        particles[self.weights < threshold][:, 2] += (randn(low_N) * std[2])
        particles[self.weights < threshold][:, 2] %= 2 * np.pi

    def step(self, u, dt, cloud: np.array, gnss_pose: np.array, grid: np.array) -> tuple:

        particles = self.create_gaussian_particles(
            [0., 0., 0.], [0., 2., 0.01], 30)

        mu, var, alignment = self.get_best_particle(particles, cloud, grid)

        pose = gnss_pose + mu

        if alignment < 20:
            pose = gnss_pose

        self.mus.append(mu)

        # if len(self.mus) % 30 == 0:
        #     mus = np.array(self.mus)
        #     plt.plot(mus[:, 0:2])
        #     plt.show()

        return pose, var

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
