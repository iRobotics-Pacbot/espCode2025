import numpy as np
from scipy.stats import norm
from .sensor_projection import sensor_projection

import time

# TODO: prior probability in predict step?
class ParticleFilter:
    def __init__(self, num_particles, initial_pose, initial_sigma, process_noise):
        # (x, y, vx, vy)
        self.particles = np.random.normal(initial_pose, initial_sigma, (num_particles, 4))
        # (dx, dy, vx, vy)
        self.odometry = np.random.normal(initial_pose, initial_sigma, (num_particles, 4))
        self.weights = np.ones(num_particles, dtype=np.float64) / num_particles
        self.process_noise = process_noise
        self.estimate = initial_pose
        self.lastPredictTime = time.perf_counter()
    
    def predict(self, timestamp):
        dt = timestamp - self.lastPredictTime
        self.lastPredictTime = time.perf_counter()
        # process noise handles imprecision in our control model (namely that there is none currently)
        self.particles = np.random.normal(self.particles, self.process_noise)
        # odometry (vx * dt, vy * dt, vx, vy)
        vel = self.particles[:,2:]
        self.odometry[:,:2] = vel * dt
        self.odometry[:,2:] = vel
        # movement (x + vx * dt, y + vy * dt, vx, vy)
        self.particles[:,:2] += vel * dt
        print(self.odometry[0])

    def update(self, tof_measurements, tof_sigma, odometry, odometry_sigma, heading, calculate_tof_measurements):
        # Calculate the expected tof measurements for each particle
        particle_poses = np.column_stack((self.particles[:,:2], np.full(self.particles.shape[0], heading)))
        expected_tof_measurements = calculate_tof_measurements(particle_poses)
        # Calculate the weights for each particle
        self.weights *= norm.pdf(tof_measurements, expected_tof_measurements, tof_sigma).prod(axis=1)
        self.weights *= norm.pdf(odometry, self.odometry, odometry_sigma).prod(axis=1)
        self.weights += 1e-200
        self.weights /= np.sum(self.weights)

        self.estimate = np.average(self.particles, axis=0)

    def resample(self):
        num_particles = self.particles.shape[0]
        cumsum_weights = np.cumsum(self.weights)
        
        # Generate all random numbers at once using linspace
        # This maintains the systematic/low-variance property
        step_size = cumsum_weights[-1] / num_particles
        u_base = np.linspace(0, (num_particles-1) * step_size, num_particles)
        u = u_base + np.random.uniform(0, step_size)
        
        # Find indices using searchsorted
        indices = np.searchsorted(cumsum_weights, u)
        
        # Ensure we don't exceed array bounds
        indices = np.clip(indices, 0, num_particles - 1)
        
        # Update particles and weights
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / num_particles)

    def get_state_estimate(self):
        return self.estimate
    
    def test(self):
        print("foo")