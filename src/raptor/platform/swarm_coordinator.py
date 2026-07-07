"""Swarm coordination for multi-drone missions."""
from dataclasses import dataclass, field
import math


@dataclass
class DroneState:
    uav_id: str
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)


@dataclass
class FormationConfig:
    spacing: float = 5.0
    pattern: str = "v_shape"


class SwarmCoordinator:
    SAFE_DISTANCE = 3.0

    def __init__(self):
        self._drones: dict[str, DroneState] = {}
        self._formation = FormationConfig()

    def register_drone(self, state: DroneState):
        self._drones[state.uav_id] = state

    def update_state(self, uav_id: str, position: tuple, velocity: tuple):
        if uav_id in self._drones:
            self._drones[uav_id].position = position
            self._drones[uav_id].velocity = velocity

    def compute_formation_offset(self, uav_id: str, leader_id: str) -> tuple | None:
        if uav_id not in self._drones or leader_id not in self._drones:
            return None
        ids = sorted(self._drones.keys())
        if uav_id not in ids or leader_id not in ids:
            return None
        idx = ids.index(uav_id)
        leader_idx = ids.index(leader_id)
        rel = idx - leader_idx
        if rel == 0:
            return (0, 0, 0)
        side = 1 if rel % 2 else -1
        row = (abs(rel) + 1) // 2
        return (row * self._formation.spacing, side * self._formation.spacing, 0)

    def collision_check(self, uav_id: str) -> list[str]:
        if uav_id not in self._drones:
            return []
        threats = []
        pos = self._drones[uav_id].position
        for uid, state in self._drones.items():
            if uid == uav_id:
                continue
            dist = math.sqrt(sum((a-b)**2 for a, b in zip(pos, state.position)))
            if dist < self.SAFE_DISTANCE:
                threats.append(uid)
        return threats

    def avoidance_velocity(self, uav_id: str) -> tuple:
        threats = self.collision_check(uav_id)
        if not threats:
            return (0, 0, 0)
        pos = self._drones[uav_id].position
        vx, vy, vz = 0.0, 0.0, 0.0
        for tid in threats:
            tp = self._drones[tid].position
            dx = pos[0] - tp[0]
            dy = pos[1] - tp[1]
            dz = pos[2] - tp[2]
            dist = max(0.1, math.sqrt(dx**2 + dy**2 + dz**2))
            strength = self.SAFE_DISTANCE / dist
            vx += dx / dist * strength
            vy += dy / dist * strength
            vz += dz / dist * strength
        n = len(threats)
        return (vx/n, vy/n, vz/n)
