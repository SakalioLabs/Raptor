import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.optimization.energy_optimizer import EnergyOptimizer, BatteryState, TrajectorySegment

def test_energy_estimation():
    eo = EnergyOptimizer()
    seg = TrajectorySegment(np.array([0,0,0]), np.array([10,0,0]), 2.0, 5.0)
    energy = eo.estimate_energy([seg])
    assert energy > 0

def test_speed_optimization_low_battery():
    eo = EnergyOptimizer()
    bat = BatteryState(voltage=22.2, capacity_mah=5000, current_mah=4500)
    seg = TrajectorySegment(np.array([0,0,0]), np.array([10,0,0]), 2.0, 5.0)
    opt = eo.optimize_speed([seg], bat)
    assert opt[0].velocity < seg.velocity

def test_remaining_range():
    eo = EnergyOptimizer()
    bat = BatteryState(voltage=22.2, capacity_mah=5000, current_mah=0)
    r = eo.estimate_remaining_range(bat, 5.0)
    assert r > 0

    bat_low = BatteryState(voltage=22.2, capacity_mah=5000, current_mah=4900)
    r_low = eo.estimate_remaining_range(bat_low, 5.0)
    assert r_low < r

def test_minimum_snap_trajectory():
    eo = EnergyOptimizer()
    wp = np.array([0, 5, 10, 5, 0])
    coeffs = eo.compute_minimum_snap_trajectory(wp, 4.0)
    assert coeffs.shape[0] == 4
