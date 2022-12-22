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

import math
import numpy as np
import scipy
import time


class MCL:

    def __init__(self, map: np.array, initial_xyh: np.array):
        self.map = map
        self.initial_xyh = initial_xyh
        self.time = time.time()

    def predict_new_poses(self, particles: np.array, s: float, dh: float, std: np.array):
        """Given speed s and heading change dh, plus 
        std [speed_std, heading_std], update the states of all particles.

        Args:
            particles (np.array): _description_
            s (float): _description_
            dh (float): _description_
            std (np.array): _description_
        """

        N = len(particles)
        dt = time.time() - self.time
        self.time = time.time()

        # update heading
        particles[:, 2] += dh*dt + (np.random.randn(N) * std[0])
        particles[:, 2] %= 2 * np.pi

        # move in the (noisy) commanded direction
        dist = (s * dt) + (np.random.randn(N) * std[1])
        particles[:, 0] += np.cos(particles[:, 2]) * dist
        particles[:, 1] += np.sin(particles[:, 2]) * dist

    def update_weights(self, particles: np.array, weights: np.array, z, R, landmarks):
        for i, landmark in enumerate(landmarks):
            distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
            weights *= scipy.stats.norm(distance, R).pdf(z[i])

        weights += 1.e-300      # avoid round-off to zero
        weights /= sum(weights)  # normalize

    def estimate(self, particles: np.array, weights: np.array):
        """Returns the mean and variance of the weighted particles

        Args:
            particles (np.array): The weighted particles.
            weights (np.array): The weights

        Returns:
            _type_: _description_
        """        """returns mean and variance of the weighted particles"""

        pos = particles[:, 0:2]
        mean = np.average(pos, weights=weights, axis=0)
        var = np.average((pos - mean)**2, weights=weights, axis=0)
        return mean, var

    def resample(self, particles, weights):
        N = len(particles)
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.  # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, np.random.random(N))

        # resample according to indexes
        particles[:] = particles[indexes]
        weights.fill(1.0 / N)

    def neff(weights):
        return 1. / np.sum(np.square(weights))
