import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.vision.depth_estimator import DepthEstimator

def test_estimate_returns_depth():
    de = DepthEstimator(model_size=(64, 64))
    frame = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    result = de.estimate(frame)
    assert result.depth_map.shape == (64, 64)
    assert result.min_depth > 0

def test_obstacles_detection():
    de = DepthEstimator(model_size=(64, 64))
    depth = np.ones((64, 64)) * 5.0
    depth[20:40, 20:40] = 1.0
    obs = de.detect_obstacles(depth, threshold=3.0)
    assert len(obs) >= 1
