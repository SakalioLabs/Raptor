"""Waypoint navigation for autonomous flight."""
from dataclasses import dataclass
from enum import Enum, auto
import math


class WaypointAction(Enum):
    NAVIGATE = auto()
    LOITER = auto()
    TAKE_PHOTO = auto()
    LAND = auto()


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float
    action: WaypointAction = WaypointAction.NAVIGATE
    loiter_time: float = 0.0


@dataclass
class Position:
    lat: float
    lon: float
    alt: float


@dataclass
class NavigationOutput:
    distance_m: float
    bearing_rad: float
    altitude_diff: float
    reached: bool


class WaypointNavigator:
    def __init__(self, acceptance_radius: float = 2.0):
        self.acceptance_radius = acceptance_radius
        self._waypoints: list[Waypoint] = []
        self._index = 0

    def set_waypoints(self, waypoints: list[Waypoint]):
        self._waypoints = list(waypoints)
        self._index = 0

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def mission_complete(self) -> bool:
        return self._index >= len(self._waypoints)

    def navigate(self, current: Position) -> NavigationOutput:
        if self.mission_complete:
            return NavigationOutput(0, 0, 0, True)
        wp = self._waypoints[self._index]
        dist = self.haversine(current.lat, current.lon, wp.lat, wp.lon)
        bearing = self.bearing(current.lat, current.lon, wp.lat, wp.lon)
        alt_diff = wp.alt - current.alt
        reached = dist < self.acceptance_radius
        if reached:
            self._index += 1
        return NavigationOutput(dist, bearing, alt_diff, reached)

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat/2)**2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2) -> float:
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(math.radians(lat2))
        x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
             math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
        return math.atan2(y, x)
