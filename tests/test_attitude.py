import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.flight.attitude import Quaternion

def test_identity_quaternion():
    q = Quaternion()
    assert q.w == 1.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0

def test_normalize():
    q = Quaternion(2, 0, 0, 0)
    n = q.normalize()
    assert abs(n.w - 1.0) < 1e-6

def test_conjugate():
    q = Quaternion(1, 2, 3, 4)
    c = q.conjugate()
    assert c.w == 1 and c.x == -2 and c.y == -3 and c.z == -4

def test_identity_multiply():
    q = Quaternion(1, 2, 3, 4).normalize()
    result = Quaternion().multiply(q)
    assert abs(result.w - q.w) < 1e-6

def test_from_euler_roundtrip():
    roll, pitch, yaw = 0.1, 0.2, 0.3
    q = Quaternion.from_euler(roll, pitch, yaw)
    r, p, y = q.to_euler()
    assert abs(r - roll) < 1e-6
    assert abs(p - pitch) < 1e-6
    assert abs(y - yaw) < 1e-6

def test_from_axis_angle():
    q = Quaternion.from_axis_angle((0, 0, 1), math.pi)
    assert abs(q.w) < 1e-6
