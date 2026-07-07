import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.mission.geofence import Geofence, GeofenceZone, GeofencePoint

def _make_zone():
    return GeofenceZone("test", [
        GeofencePoint(0, 0), GeofencePoint(0, 1),
        GeofencePoint(1, 1), GeofencePoint(1, 0)
    ], 0, 120)

def test_point_inside():
    gf = Geofence()
    gf.add_zone(_make_zone())
    check = gf.check_position(0.5, 0.5, 50)
    assert check.inside
    assert check.altitude_ok

def test_point_outside():
    gf = Geofence()
    gf.add_zone(_make_zone())
    check = gf.check_position(5, 5, 50)
    assert not check.inside

def test_altitude_violation():
    gf = Geofence()
    gf.add_zone(_make_zone())
    check = gf.check_position(0.5, 0.5, 200)
    assert not check.altitude_ok
