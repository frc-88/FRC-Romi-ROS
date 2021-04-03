import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats


class ParticleFilter(object):
    def __init__(self, num_particles, measure_std_error, input_std_error):
        self.num_states = 3  # x, y, theta
        self.particles = np.zeros((num_particles, self.num_states))
        self.num_particles = num_particles
        self.measure_std_error = measure_std_error
        self.input_std_error = np.array(input_std_error)

        self.measure_distribution = scipy.stats.norm(0.0, self.measure_std_error)

        self.expected_measurement_container = np.zeros(num_particles)

        self.initialize_weights()
    
    def is_initialized(self):
        return not np.all(self.particles == 0.0)

    def initialize_weights(self):
        # initialize with uniform weight
        self.weights = np.ones(self.num_particles)
        self.weights /= np.sum(self.weights)

    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        self.initialize_weights()
        for state_num in range(self.num_states):
            min_val = initial_state[state_num] - state_range[state_num]
            max_val = initial_state[state_num] + state_range[state_num]
            self.particles[:, state_num] = uniform(min_val, max_val, size=self.num_particles)

    def create_gaussian_particles(self, mean, var):
        self.initialize_weights()
        for state_num in range(self.num_states):
            self.particles[:, state_num] = mean[state_num] + randn(self.num_particles) * var[state_num]

    def predict(self, u, dt):
        """
        move according to control input u (linear vx, angular v) with noise std
        """
        self.particles[:, 0] += u[0] * dt * np.cos(u[1]) + randn(self.num_particles) * self.input_std_error[0]
        self.particles[:, 1] += u[0] * dt * np.sin(u[1]) + randn(self.num_particles) * self.input_std_error[0]
        self.particles[:, 2] += u[1] * dt + randn(self.num_particles) * self.input_std_error[1]

    def update(self, z, sensor_serial):
        """Update particle filter according to measurement z (a single measurement from one of the distance sensors)"""
        # weight according to how far away the particle is from the measurement in x, y, z
        # self.weights.fill(1.0)

        errors = self.expected_measurement(sensor_serial) - z
        self.weights *= self.measure_distribution.pdf(errors)

        self.weights += 1.e-300  # avoid divide by zero error
        self.weights /= np.sum(self.weights)  # normalize

    def expected_measurement(self, sensor_serial):
        """
        Based on the position of each particle and where the sensor is mounted,
        calculate what the measurement should be based on a stored map.
        """
        for index, state in enumerate(self.particles):
            self.expected_measurement_container[index] = self.map_raytracer.trace(state, sensor_serial)
        return self.expected_measurement_container

    def neff(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample(self):
        # indices = self.simple_resample()
        indices = self.systematic_resample()

        # resample according to indices
        self.particles = self.particles[indices]
        self.weights = self.weights[indices]
        self.weights /= np.sum(self.weights)  # normalize

    def resample_from_index(self, indices):
        assert len(indices) == self.num_particles

        self.particles = self.particles[indices]
        self.weights = self.weights[indices]
        self.weights /= np.sum(self.weights)

    def estimate(self):
        """ returns mean and variance """
        mu = self.mean()
        var = np.average((self.particles - mu) ** 2, weights=self.weights, axis=0)

        return mu, var

    def mean(self):
        """ returns weighted mean position"""
        return np.average(self.particles, weights=self.weights, axis=0)

    def check_resample(self):
        neff = self.neff()
        # print "neff:", neff
        if neff < self.num_particles / 2.0:
            self.resample()
            return True
        else:
            return False

    def simple_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # avoid round-off error
        indices = np.searchsorted(cumulative_sum, random(self.num_particles))
        return indices

    def systematic_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        indices = np.zeros(self.num_particles, 'int')
        t = np.linspace(0, 1.0 - 1.0 / self.num_particles, self.num_particles) + random() / self.num_particles

        i, j = 0, 0
        while i < self.num_particles and j < self.num_particles:
            while cumulative_sum[j] < t[i]:
                j += 1
            indices[i] = j
            i += 1

        return indices
