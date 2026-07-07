import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.flight.motor_mixer import HexacopterMixer

def test_hover():
    m = HexacopterMixer()
    out = m.mix(0.5, 0, 0, 0)
    for v in out.as_list():
        assert 0.4 < v < 0.6

def test_output_range():
    m = HexacopterMixer()
    out = m.mix(1.0, 1.0, 1.0, 1.0)
    for v in out.as_list():
        assert 0.0 <= v <= 1.0

def test_zero_throttle():
    m = HexacopterMixer()
    out = m.mix(0.0, 0, 0, 0)
    assert all(v == 0.0 for v in out.as_list())
