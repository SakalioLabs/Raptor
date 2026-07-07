import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.platform.swarm_coordinator import SwarmCoordinator, DroneState

def test_register_drone():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("uav1", (0, 0, 0), (0, 0, 0)))
    assert "uav1" in sc._drones

def test_no_collision_far():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("uav1", (0, 0, 0), (0, 0, 0)))
    sc.register_drone(DroneState("uav2", (10, 10, 10), (0, 0, 0)))
    assert sc.collision_check("uav1") == []

def test_collision_near():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("uav1", (0, 0, 0), (0, 0, 0)))
    sc.register_drone(DroneState("uav2", (1, 0, 0), (0, 0, 0)))
    threats = sc.collision_check("uav1")
    assert "uav2" in threats

def test_avoidance_velocity():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("uav1", (0, 0, 0), (0, 0, 0)))
    sc.register_drone(DroneState("uav2", (1, 0, 0), (0, 0, 0)))
    v = sc.avoidance_velocity("uav1")
    assert v[0] < 0

def test_formation_offset():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("a", (0,0,0), (0,0,0)))
    sc.register_drone(DroneState("b", (5,0,0), (0,0,0)))
    offset = sc.compute_formation_offset("b", "a")
    assert offset is not None
    assert offset[0] > 0

def test_no_avoidance_far():
    sc = SwarmCoordinator()
    sc.register_drone(DroneState("uav1", (0, 0, 0), (0, 0, 0)))
    sc.register_drone(DroneState("uav2", (100, 100, 100), (0, 0, 0)))
    v = sc.avoidance_velocity("uav1")
    assert v == (0, 0, 0)
