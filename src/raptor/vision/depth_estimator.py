"""Monocular depth estimation wrapper."""
from dataclasses import dataclass
import numpy as np


@dataclass
class DepthResult:
    depth_map: np.ndarray
    min_depth: float
    max_depth: float
    mean_depth: float


class DepthEstimator:
    """Simulates depth estimation from a single camera frame."""
    def __init__(self, model_size=(256, 256)):
        self.model_size = model_size

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        h, w = self.model_size
        from scipy.ndimage import zoom
        sy = h / frame.shape[0]
        sx = w / frame.shape[1]
        if frame.ndim == 3:
            resized = zoom(frame, (sy, sx, 1), order=1)
        else:
            resized = zoom(frame, (sy, sx), order=1)
        return resized.astype(np.float32) / 255.0

    def estimate(self, frame: np.ndarray) -> DepthResult:
        processed = self.preprocess(frame)
        if processed.ndim == 3:
            gray = np.mean(processed, axis=2)
        else:
            gray = processed
        depth = 1.0 / (gray + 0.1)
        return DepthResult(
            depth_map=depth,
            min_depth=float(np.min(depth)),
            max_depth=float(np.max(depth)),
            mean_depth=float(np.mean(depth)),
        )

    def detect_obstacles(self, depth_map: np.ndarray, threshold: float = 3.0) -> list:
        mask = depth_map < threshold
        rows = np.any(mask, axis=1)
        if not np.any(rows):
            return []
        top = int(np.argmax(rows))
        bottom = len(rows) - 1 - np.argmax(rows[::-1])
        cols = np.any(mask, axis=0)
        left = int(np.argmax(cols))
        right = len(cols) - 1 - np.argmax(cols[::-1])
        region = depth_map[top:bottom+1, left:right+1]
        avg_dist = float(np.mean(region))
        return [{"x": left, "y": top, "w": right-left, "h": bottom-top, "distance": avg_dist}]
