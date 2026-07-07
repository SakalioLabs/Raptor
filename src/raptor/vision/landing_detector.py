"""Precision landing detection using ArUco-style markers."""
from dataclasses import dataclass
import math


@dataclass
class MarkerDetection:
    marker_id: int
    cx: float
    cy: float
    distance: float
    angle: float


@dataclass
class LandingCommand:
    offset_x: float = 0.0
    offset_y: float = 0.0
    altitude: float = 0.0
    ready: bool = False


class LandingDetector:
    def __init__(self, approach_alt: float = 2.0, touch_down_alt: float = 0.3,
                 center_threshold: float = 0.1):
        self.approach_alt = approach_alt
        self.touch_down_alt = touch_down_alt
        self.center_threshold = center_threshold

    def detect(self, markers: list, altitude: float) -> LandingCommand:
        if not markers:
            return LandingCommand()
        best = min(markers, key=lambda m: abs(m.cx) + abs(m.cy))
        ready = (altitude <= self.touch_down_alt
                 and abs(best.cx) < self.center_threshold
                 and abs(best.cy) < self.center_threshold)
        return LandingCommand(best.cx, best.cy, altitude, ready)
