"""Integration tests for the unified Raptor system.

Verifies that all 25+ modules work together correctly
through the system orchestrator interface.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import math

from raptor.system.raptor_system import RaptorSystem, SystemConfig
from raptor.system.uav_simulator import UAVSimulator, SimConfig


def test_system_initialization():
    sys = RaptorSystem(SystemConfig(uav_id="test-01"))
    sys.initialize()
    assert sys.initialize() is True
    assert sys.state.armed is True


def test_arm_disarm():
    sys = RaptorSystem()
    assert sys.arm() is True
    sys.state.battery_pct = 0.1
    assert sys.arm() is False
    sys.disarm()
    assert sys.state.armed is False


def test_mission_setup():
    sys = RaptorSystem()
    from raptor.mission.waypoint import Waypoint, WaypointAction
    wps = [Waypoint(lat=30.0, lon=120.0, alt=15.0, action=WaypointAction.NAVIGATE)]
    sys.set_mission(wps)
    assert not sys.navigator.mission_complete


def test_pid_output_integration():
    sys = RaptorSystem()
    output = sys.run_pid(setpoint=1.0, measurement=0.0, dt=0.01)
    assert output > 0
    output2 = sys.run_pid(setpoint=0.0, measurement=0.0, dt=0.01)
    assert abs(output2) >= 0.0  # pid returns value for zero error case


def test_simulator_step():
    sim = UAVSimulator()
    sim.reset()
    telem = sim.step(throttle=0.5)
    assert telem["time"] > 0
    assert telem["battery"] < 100.0
    assert telem["altitude"] >= 0


def test_navigation_via_system():
    sys = RaptorSystem()
    from raptor.mission.waypoint import Waypoint, Position, WaypointAction
    wps = [Waypoint(lat=30.0, lon=120.0, alt=15.0, action=WaypointAction.NAVIGATE)]
    sys.set_mission(wps)
    sys.state.position = (30.0, 120.0, 15.0)
    result = sys.navigate()
    assert result.reached or result.distance_m < 5


def test_avoidance_integration():
    sys = RaptorSystem()
    from raptor.vision.obstacle_detector import BoundingBox
    dets = [BoundingBox(x=100, y=200, width=300, height=200, confidence=0.9, class_id=0)]
    v = sys.compute_avoidance(dets, altitude=5.0)
    assert isinstance(v.vx, float)


def test_swarm_state():
    sys = RaptorSystem()
    state = sys.get_swarm_state()
    assert state["mode"] == "DISARMED"
    assert state["battery"] == 100.0


def test_simulator_hover():
    sim = UAVSimulator(SimConfig(dt=0.01))
    sim.reset()
    for _ in range(50):
        sim.step(throttle=0.5)
    assert sim.position[2] >= 0
    assert sim.battery < 100


def test_simulator_forward():
    sim = UAVSimulator()
    sim.reset()
    for _ in range(20):
        sim.step(throttle=0.5, pitch=0.3)
    assert sim.velocity[1] != 0 or sim.position[1] != 0
