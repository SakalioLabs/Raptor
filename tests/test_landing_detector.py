import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.vision.landing_detector import LandingDetector, MarkerDetection

def test_no_markers():
    ld = LandingDetector()
    cmd = ld.detect([], 5.0)
    assert not cmd.ready

def test_landing_ready():
    ld = LandingDetector(touch_down_alt=0.3, center_threshold=0.1)
    marker = MarkerDetection(marker_id=0, cx=0.0, cy=0.0, distance=0.5, angle=0.0)
    cmd = ld.detect([marker], 0.2)
    assert cmd.ready

def test_not_ready_high_altitude():
    ld = LandingDetector(touch_down_alt=0.3)
    marker = MarkerDetection(marker_id=0, cx=0.0, cy=0.0, distance=0.5, angle=0.0)
    cmd = ld.detect([marker], 2.0)
    assert not cmd.ready
