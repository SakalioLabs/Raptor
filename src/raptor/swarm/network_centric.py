"""Network-Centric UAV Management (网络协管无人机).

Implements hierarchical command and control for UAV swarms based on
network-centric warfare principles. Supports dynamic role assignment,
distributed situational awareness, and adaptive task reassignment.

Based on: Alberts et al., "Network Centric Warfare" (2002),
and Zhejiang University research on intelligent network management
for cooperative UAV systems.
"""
from dataclasses import dataclass, field
from enum import Enum, auto
from collections import defaultdict
import math


class Role(Enum):
    COMMANDER = auto()
    SCOUT = auto()
    STRIKER = auto()
    RELAY = auto()
    SUPPORT = auto()


class TaskStatus(Enum):
    PENDING = auto()
    ASSIGNED = auto()
    IN_PROGRESS = auto()
    COMPLETED = auto()
    FAILED = auto()


@dataclass
class UAVNode:
    uav_id: str
    role: Role = Role.SUPPORT
    position: tuple = (0, 0, 0)
    battery_pct: float = 1.0
    capabilities: list = field(default_factory=list)
    connected_peers: set = field(default_factory=set)


@dataclass
class MissionTask:
    task_id: str
    position: tuple
    priority: float = 1.0
    required_role: Role = Role.STRIKER
    status: TaskStatus = TaskStatus.PENDING
    assigned_to: str = None


class NetworkCentricManager:
    def __init__(self, max_hops: int = 3):
        self.uavs: dict[str, UAVNode] = {}
        self.tasks: dict[str, MissionTask] = {}
        self.max_hops = max_hops
        self._awareness_graph: dict[str, dict] = defaultdict(dict)

    def register_uav(self, uav: UAVNode):
        self.uavs[uav.uav_id] = uav

    def add_connection(self, uav_id1: str, uav_id2: str):
        if uav_id1 in self.uavs and uav_id2 in self.uavs:
            self.uavs[uav_id1].connected_peers.add(uav_id2)
            self.uavs[uav_id2].connected_peers.add(uav_id1)

    def submit_task(self, task: MissionTask):
        self.tasks[task.task_id] = task

    def get_situational_awareness(self, uav_id: str) -> dict:
        if uav_id not in self.uavs:
            return {}
        sa = {"peers": {}, "tasks": {}, "threats": []}
        visited = set()
        queue = [(uav_id, 0)]
        while queue:
            current, depth = queue.pop(0)
            if current in visited or depth > self.max_hops:
                continue
            visited.add(current)
            if current != uav_id and current in self.uavs:
                peer = self.uavs[current]
                sa["peers"][current] = {
                    "position": peer.position,
                    "role": peer.role.name,
                    "battery": peer.battery_pct,
                }
            for peer_id in self.uavs.get(current, UAVNode("")).connected_peers:
                if peer_id not in visited:
                    queue.append((peer_id, depth + 1))
        for tid, task in self.tasks.items():
            dist = self._distance(
                self.uavs[uav_id].position, task.position
            )
            sa["tasks"][tid] = {
                "position": task.position,
                "priority": task.priority,
                "status": task.status.name,
                "distance": dist,
            }
        return sa

    def assign_tasks(self) -> dict:
        assignments = {}
        pending = [t for t in self.tasks.values()
                   if t.status == TaskStatus.PENDING]
        pending.sort(key=lambda t: t.priority, reverse=True)
        available = [u for u in self.uavs.values()
                     if self._is_available(u)]
        for task in pending:
            best_uav = None
            best_score = -1
            for uav in available:
                score = self._assignment_score(uav, task)
                if score > best_score:
                    best_score = score
                    best_uav = uav
            if best_uav:
                task.status = TaskStatus.ASSIGNED
                task.assigned_to = best_uav.uav_id
                assignments[task.task_id] = best_uav.uav_id
                available.remove(best_uav)
        return assignments

    def update_roles(self):
        for uav_id, uav in self.uavs.items():
            if uav.battery_pct < 0.2:
                uav.role = Role.RELAY
            elif uav.battery_pct < 0.4:
                uav.role = Role.SUPPORT
            peers = len(uav.connected_peers)
            if peers >= len(self.uavs) * 0.5 and uav.battery_pct > 0.7:
                uav.role = Role.COMMANDER

    def _assignment_score(self, uav: UAVNode, task: MissionTask) -> float:
        dist = self._distance(uav.position, task.position)
        dist_score = 1.0 / max(dist, 1.0)
        role_match = 1.0 if uav.role == task.required_role else 0.3
        battery_score = uav.battery_pct
        return (dist_score * 0.4 + role_match * 0.4 + battery_score * 0.2) * task.priority

    def _is_available(self, uav: UAVNode) -> bool:
        for task in self.tasks.values():
            if task.assigned_to == uav.uav_id and task.status in (
                TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS
            ):
                return False
        return uav.battery_pct > 0.15

    @staticmethod
    def _distance(p1: tuple, p2: tuple) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))
