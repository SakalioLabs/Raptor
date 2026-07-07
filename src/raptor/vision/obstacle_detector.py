"""Obstacle detection using bounding box analysis."""
from dataclasses import dataclass
import math


@dataclass
class BoundingBox:
    x: float
    y: float
    width: float
    height: float
    confidence: float
    class_id: int


@dataclass
class AvoidanceVector:
    vx: float = 0.0
    vy: float = 0.0
    threat_level: float = 0.0


class ObstacleDetector:
    """Compute avoidance vectors from detected bounding boxes."""
    def __init__(self, image_width: int = 640, image_height: int = 640,
                 danger_zone: float = 0.3, fov_deg: float = 72.0):
        self.image_width = image_width
        self.image_height = image_height
        self.danger_zone = danger_zone
        self.fov_rad = math.radians(fov_deg)

    def compute_avoidance(self, detections: list, estimated_distance: float) -> AvoidanceVector:
        if not detections:
            return AvoidanceVector()
        vx, vy, max_threat = 0.0, 0.0, 0.0
        for det in detections:
            cx = (det.x + det.width / 2) / self.image_width
            cy = (det.y + det.height / 2) / self.image_height
            norm_x = cx - 0.5
            area = (det.width * det.height) / (self.image_width * self.image_height)
            threat = det.confidence * min(1.0, area / self.danger_zone)
            dist_factor = max(0.1, min(1.0, estimated_distance / 5.0))
            threat *= (1.0 / dist_factor)
            max_threat = max(max_threat, threat)
            direction = -1.0 if norm_x > 0 else 1.0
            vx += direction * threat * abs(norm_x) * 2
            vy -= threat * (cy - 0.5)
        n = len(detections)
        return AvoidanceVector(vx/n, vy/n, min(1.0, max_threat))
