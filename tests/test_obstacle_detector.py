import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.vision.obstacle_detector import ObstacleDetector, BoundingBox

def test_no_detections():
    det = ObstacleDetector()
    v = det.compute_avoidance([], 5.0)
    assert v.vx == 0.0 and v.vy == 0.0

def test_center_obstacle():
    det = ObstacleDetector(image_width=640, image_height=640)
    box = BoundingBox(x=220, y=220, width=200, height=200, confidence=0.9, class_id=0)
    v = det.compute_avoidance([box], 2.0)
    assert v.threat_level > 0

def test_left_obstacle_pushes_right():
    det = ObstacleDetector(image_width=640, image_height=640)
    box = BoundingBox(x=0, y=0, width=100, height=100, confidence=0.8, class_id=0)
    v = det.compute_avoidance([box], 2.0)
    assert v.vx >= 0
