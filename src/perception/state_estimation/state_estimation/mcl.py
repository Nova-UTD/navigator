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
This code is adapted from Roger Labbe: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/

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

    def update_weights(self, particles, weights, cloud: np.array, cells: np.array):
        """Update weights by checking each particle's alignment in the occupancy grid.

        Args:
            particles (_type_): _description_
            weights (_type_): _description_
            cloud (np.array): [[x,y], [x,y], ...]
            cells (np.array): Each cell is either 0 or 1, where 1 is occupied.
        """

        # The cloud is given in the vehicle frame.
        # We need it in the map frame
        alignments = []

        for particle in particles:

            # First rotate
            theta = particle[2]
            r = np.array([[np.cos(theta), -1*np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])

            transformed_cloud = np.dot(cloud, r.T)

            # Then translate
            transformed_cloud[:] += particle[0:2]

            # Translate relative to map origin
            transformed_cloud = np.subtract(transformed_cloud, self.map_origin)

            # Round each point in the cloud down to an int
            # Now each point represents an index in cells. Convenient!
            grid_indices = transformed_cloud.astype(np.int8)

            hits = 0

            for index in grid_indices[::10]:
                if cells[index[0], index[1]] == 100:
                    # print("Hit!")
                    hits += 1
                # elif cells[index[0], index[1]] != 0:
                #     print(cells[index[0], index[1]])

            alignment = hits / len(cloud)
            alignments.append(alignment)

        # print(alignments)

        weights *= alignments

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

        # pos = particles[:, 0:2]
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

    def __init__(self, grid: np.array, initial_pose=np.array([0.0, 0.0, 0.0]), map_origin=np.array([0.0, 0.0]), N=100):
        self.particles = self.create_gaussian_particles(
            mean=initial_pose, std=(5, 5, np.pi/4), N=N)

        self.weights = np.ones(N) / N

        self.grid = grid
        self.map_origin = map_origin

    def step(self, delta: np.array, cloud: np.array) -> tuple:
        self.predict(self.particles, delta, std=np.array([1.0, 1.0, 0.2]))

        self.update_weights(self.particles, self.weights, cloud, self.grid)

        # Determine if a resample is necessary
        # N/2 is a good threshold
        N = len(self.particles)
        if self.neff(self.weights) < N/2:
            indexes = self.systematic_resample(self.weights)
            self.resample_from_index(self.particles, self.weights, indexes)
            assert np.allclose(self.weights, 1/N)
        mu, var = self.estimate(self.particles, self.weights)

        return mu, var

    def run_pf1(self, N, iters=18, sensor_std_err=.1,
                plot_particles=False,
                xlim=(0, 20), ylim=(0, 20),
                initial_x=None):
        landmarks = np.array([[-1, 2], [5, 10], [12, 14], [18, 21]])
        NL = len(landmarks)

        plt.figure()

        # create particles and weights
        if initial_x is not None:
            particles: np.array = self.create_gaussian_particles(
                mean=initial_x, std=(5, 5, np.pi/4), N=N)
        else:
            particles: np.array = self.create_uniform_particles(
                (0, 20), (0, 20), (0, 6.28), N)
        weights = np.ones(N) / N

        if plot_particles:
            alpha = .20
            if N > 5000:
                alpha *= np.sqrt(5000)/np.sqrt(N)
            plt.scatter(particles[:, 0], particles[:, 1],
                        alpha=alpha, color='g')

        xs = []
        robot_pos = np.array([0., 0.])
        for x in range(iters):
            robot_pos += (1, 1)

            # distance from robot to each landmark
            zs = (np.linalg.norm(landmarks - robot_pos, axis=1) +
                  (randn(NL) * sensor_std_err))

            # move diagonally forward to (x+1, x+1)
            self.predict(particles, u=(0.00, 1.414), std=(.2, .05))

            # incorporate measurements
            self.update_orig(particles, weights, z=zs, R=sensor_std_err,
                             landmarks=landmarks)

            # resample if too few effective particles
            if self.neff(weights) < N/2:
                indexes = self.systematic_resample(weights)
                self.resample_from_index(particles, weights, indexes)
                assert np.allclose(weights, 1/N)
            mu, var = self.estimate(particles, weights)
            xs.append(mu)

            if plot_particles:
                plt.scatter(particles[:, 0], particles[:, 1],
                            color='k', marker=',', s=1)
            p1 = plt.scatter(robot_pos[0], robot_pos[1], marker='+',
                             color='k', s=180, lw=3)
            p2 = plt.scatter(mu[0], mu[1], marker='s', color='r')

        xs = np.array(xs)
        #plt.plot(xs[:, 0], xs[:, 1])
        plt.legend([p1, p2], ['Actual', 'PF'], loc=4, numpoints=1)
        plt.xlim(*xlim)
        plt.ylim(*ylim)
        print('final position error, variance:\n\t',
              mu - np.array([iters, iters]), var)
        plt.show()

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


if __name__ == "__main__":
    seed(2)
    mcl = MCL(np.array([0.0, 0.0, 0.0]), np.zeros((254, 254)))
    mcl.run_pf1(N=5000, plot_particles=False)
