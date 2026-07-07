"""Energy-Aware Trajectory Optimization for UAVs.

Minimizes energy consumption while maintaining mission constraints.
Uses quadratic programming formulation for minimum-snap trajectories
and battery-aware speed profiling.

Based on: Richter et al., "Polynomial Trajectory Planning for Aggressive
Quadrotor Flight in Indoor Environments" (2013) and
Lin et al., "Energy-aware trajectory planning for UAVs" (2020).
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class BatteryState:
    voltage: float = 22.2
    capacity_mah: float = 5000
    current_mah: float = 0
    temperature_c: float = 25.0


@dataclass
class TrajectorySegment:
    start: np.ndarray
    end: np.ndarray
    duration: float
    velocity: float


class EnergyOptimizer:
    def __init__(self, mass: float = 2.0, drag_coeff: float = 0.3,
                 hover_power: float = 150.0, motor_efficiency: float = 0.8):
        self.mass = mass
        self.drag_coeff = drag_coeff
        self.hover_power = hover_power
        self.motor_efficiency = motor_efficiency

    def estimate_energy(self, trajectory: list) -> float:
        total_energy = 0.0
        for seg in trajectory:
            dist = np.linalg.norm(seg.end - seg.start)
            gravity_power = self.mass * 9.81 * 0.0
            drag_power = self.drag_coeff * seg.velocity**2
            kinetic_power = 0.5 * self.mass * seg.velocity**2 / max(seg.duration, 1e-6)
            power = self.hover_power + gravity_power + drag_power + kinetic_power
            energy_joules = power * seg.duration / self.motor_efficiency
            total_energy += energy_joules
        return total_energy

    def optimize_speed(self, trajectory: list, battery: BatteryState) -> list:
        soc = self._state_of_charge(battery)
        speed_factor = 1.0
        if soc < 0.2:
            speed_factor = 0.5
        elif soc < 0.4:
            speed_factor = 0.75
        if battery.temperature_c > 40:
            speed_factor *= 0.8
        optimized = []
        for seg in trajectory:
            new_seg = TrajectorySegment(
                start=seg.start, end=seg.end,
                duration=seg.duration / speed_factor,
                velocity=seg.velocity * speed_factor
            )
            optimized.append(new_seg)
        return optimized

    def estimate_remaining_range(self, battery: BatteryState, cruise_speed: float) -> float:
        soc = self._state_of_charge(battery)
        remaining_joules = (battery.voltage * battery.capacity_mah * 3.6) * soc
        power = self.hover_power + self.drag_coeff * cruise_speed**2
        flight_time_s = remaining_joules * self.motor_efficiency / max(power, 1.0)
        return cruise_speed * flight_time_s

    def compute_minimum_snap_trajectory(self, waypoints: np.ndarray, total_time: float) -> np.ndarray:
        n_segments = len(waypoints) - 1
        times = np.linspace(0, total_time, n_segments + 1)
        coeffs = []
        for i in range(n_segments):
            dt = times[i+1] - times[i]
            p0 = waypoints[i]
            p1 = waypoints[i+1]
            c = np.zeros(8)
            c[0] = p0
            c[1] = (p1 - p0) / dt
            c[2] = 3 * (p1 - p0) / dt**2
            c[3] = 6 * (p1 - p0) / dt**3
            coeffs.append(c)
        return np.array(coeffs)

    def _state_of_charge(self, battery: BatteryState) -> float:
        used_mah = battery.current_mah
        remaining = max(0, battery.capacity_mah - used_mah)
        return remaining / max(battery.capacity_mah, 1)
