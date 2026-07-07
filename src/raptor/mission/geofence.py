"""Geofence enforcement for UAV safety."""
from dataclasses import dataclass
import math


@dataclass
class GeofencePoint:
    lat: float
    lon: float


@dataclass
class GeofenceZone:
    name: str
    polygon: list[GeofencePoint]
    min_alt: float = 0.0
    max_alt: float = 120.0


@dataclass
class FenceCheck:
    inside: bool
    altitude_ok: bool
    distance_to_edge: float


class Geofence:
    def __init__(self):
        self._zones: list[GeofenceZone] = []

    def add_zone(self, zone: GeofenceZone):
        self._zones.append(zone)

    def check_position(self, lat: float, lon: float, alt: float) -> FenceCheck:
        inside = False
        min_edge_dist = float('inf')
        for zone in self._zones:
            if self._point_in_polygon(lat, lon, zone.polygon):
                inside = True
            d = self._distance_to_polygon(lat, lon, zone.polygon)
            min_edge_dist = min(min_edge_dist, d)
        altitude_ok = any(z.min_alt <= alt <= z.max_alt for z in self._zones) if self._zones else True
        return FenceCheck(inside, altitude_ok, min_edge_dist)

    @staticmethod
    def _point_in_polygon(lat: float, lon: float, polygon: list[GeofencePoint]) -> bool:
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            pi, pj = polygon[i], polygon[j]
            if ((pi.lat > lat) != (pj.lat > lat) and
                lon < (pj.lon - pi.lon) * (lat - pi.lat) / (pj.lat - pi.lat) + pi.lon):
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _distance_to_polygon(lat: float, lon: float, polygon: list[GeofencePoint]) -> float:
        min_d = float('inf')
        for p in polygon:
            d = math.sqrt((p.lat - lat)**2 + (p.lon - lon)**2)
            min_d = min(min_d, d)
        return min_d * 111320
