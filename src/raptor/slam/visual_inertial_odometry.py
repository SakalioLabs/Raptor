"""Visual-Inertial Odometry (VIO) using Extended Kalman Filter.

Fuses IMU (accelerometer + gyroscope) with visual feature measurements
for robust pose estimation in GPS-denied environments.

Based on: Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter
for Vision-aided Inertial Navigation" (2007).
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class IMUMeasurement:
    accel: np.ndarray  # (3,) m/s^2
    gyro: np.ndarray   # (3,) rad/s
    dt: float          # seconds


@dataclass
class VisualMeasurement:
    feature_position: np.ndarray  # (3,) estimated feature in camera frame
    confidence: float


class EKFVIO:
    def __init__(self):
        self.state = np.zeros(16)
        self.state[6] = 1.0
        self.P = np.eye(15) * 0.01
        self.Q_imu = np.eye(15) * 0.1
        self.R_vis = np.eye(3) * 0.5
        self._gravity = np.array([0, 0, 9.81])
        self.initialized = False

    @property
    def position(self):
        return self.state[:3].copy()

    @property
    def velocity(self):
        return self.state[3:6].copy()

    def predict(self, imu: IMUMeasurement):
        accel = imu.accel - self._gravity
        dt = imu.dt
        self.state[0:3] += self.state[3:6] * dt + 0.5 * accel * dt**2
        self.state[3:6] += accel * dt
        F = np.eye(15)
        F[:3, 3:6] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + self.Q_imu
        self.initialized = True

    def update_visual(self, vis: VisualMeasurement):
        if not self.initialized:
            return
        z = vis.feature_position
        H = np.zeros((3, 15))
        H[:3, :3] = np.eye(3)
        S = H @ self.P @ H.T + self.R_vis / max(vis.confidence, 0.01)
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - self.state[:3]
        self.state[:15] += K @ y
        self.P = (np.eye(15) - K @ H) @ self.P

    def get_covariance_trace(self):
        return np.trace(self.P[:3, :3])
