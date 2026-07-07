"""Model Predictive Control (MPC) for trajectory tracking.

Implements a linear MPC formulation for quadrotor position control.
Based on: Camacho & Bordons, "Model Predictive Control" (2007)
and Richter et al., "Real-Time Trajectory Generation for Quadrotors" (2013).
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class MPCConfig:
    prediction_horizon: int = 20
    control_horizon: int = 10
    dt: float = 0.05
    Q: float = 1.0
    R: float = 0.01
    position_min: tuple = (-100, -100, 0)
    position_max: tuple = (100, 100, 120)
    velocity_max: float = 15.0
    acceleration_max: float = 5.0


@dataclass
class MPCState:
    position: np.ndarray  # (3,)
    velocity: np.ndarray  # (3,)


class MPCController:
    def __init__(self, config: MPCConfig = None):
        self.cfg = config or MPCConfig()
        self._A = np.eye(6)
        self._A[:3, 3:] = np.eye(3) * self.cfg.dt
        self._B = np.zeros((6, 3))
        self._B[3:, :] = np.eye(3) * self.cfg.dt

    def compute_control(self, current: MPCState, reference: list) -> np.ndarray:
        if len(reference) < self.cfg.prediction_horizon:
            reference = reference + [reference[-1]] * (self.cfg.prediction_horizon - len(reference))
        x = np.concatenate([current.position, current.velocity])
        u_opt = self._solve_mpc(x, reference[:self.cfg.prediction_horizon])
        return u_opt

    def _solve_mpc(self, x0, refs):
        N = self.cfg.prediction_horizon
        u_seq = np.zeros((N, 3))
        x = x0.copy()
        for i in range(min(N, self.cfg.control_horizon)):
            ref_pos = np.array(refs[i]) if not isinstance(refs[i], np.ndarray) else refs[i]
            pos_err = ref_pos - x[:3]
            vel_cmd = np.clip(pos_err * self.cfg.Q / self.cfg.R, -self.cfg.velocity_max, self.cfg.velocity_max)
            u_seq[i] = (vel_cmd - x[3:]) / max(self.cfg.dt, 1e-6)
            u_seq[i] = np.clip(u_seq[i], -self.cfg.acceleration_max, self.cfg.acceleration_max)
            x = self._A @ x + self._B @ u_seq[i]
        return u_seq[0]

    def predict_trajectory(self, x0, u_seq):
        trajectory = [x0[:3].copy()]
        x = x0.copy()
        for u in u_seq:
            x = self._A @ x + self._B @ u
            trajectory.append(x[:3].copy())
        return np.array(trajectory)
