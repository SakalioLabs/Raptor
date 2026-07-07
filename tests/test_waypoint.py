import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.mission.waypoint import WaypointNavigator, Waypoint, Position

def test_set_waypoints():
    nav = WaypointNavigator()
    nav.set_waypoints([Waypoint(30.0, 120.0, 10.0)])
    assert not nav.mission_complete

def test_navigation_reaches_waypoint():
    nav = WaypointNavigator(acceptance_radius=100000)
    nav.set_waypoints([Waypoint(30.0, 120.0, 10.0)])
    result = nav.navigate(Position(30.0, 120.0, 10.0))
    assert result.reached
    assert nav.mission_complete

def test_multi_waypoint():
    nav = WaypointNavigator(acceptance_radius=100000)
    nav.set_waypoints([Waypoint(30, 120, 10), Waypoint(31, 121, 20)])
    nav.navigate(Position(30, 120, 10))
    assert nav.current_index == 1
    nav.navigate(Position(31, 121, 20))
    assert nav.mission_complete

def test_haversine():
    d = WaypointNavigator.haversine(0, 0, 0, 1)
    assert 110000 < d < 112000
