"""UAV Simulator for testing integrated system behavior."""
from dataclasses import dataclass
import math


@dataclass
class SimConfig:
    dt: float = 0.05
    max_speed: float = 15.0
    max_accel: float = 5.0
    gravity: float = 9.81


class UAVSimulator:
    def __init__(self, config: SimConfig = None):
        self.cfg = config or SimConfig()
        self.time = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.battery = 100.0

    def reset(self):
        self.time = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.battery = 100.0

    def step(self, throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        t = max(0.0, min(1.0, throttle))
        ax = max(-1.0, min(1.0, pitch)) * self.cfg.max_accel
        ay = max(-1.0, min(1.0, roll)) * self.cfg.max_accel
        az = t * self.cfg.max_accel - self.cfg.gravity
        self.velocity[0] += ay * self.cfg.dt
        self.velocity[1] += ax * self.cfg.dt
        self.velocity[2] += az * self.cfg.dt
        self.position[0] += self.velocity[0] * self.cfg.dt
        self.position[1] += self.velocity[1] * self.cfg.dt
        self.position[2] += self.velocity[2] * self.cfg.dt
        self.position[2] = max(0.0, self.position[2])
        self.time += self.cfg.dt
        hover_power = 150.0
        self.battery -= hover_power * self.cfg.dt / 3600 / 11.1 * 100
        self.battery = max(0.0, self.battery)
        return self.get_telemetry()

    def get_telemetry(self):
        return {"position": tuple(self.position), "velocity": tuple(self.velocity),
                "battery": self.battery, "altitude": self.position[2], "time": self.time}
