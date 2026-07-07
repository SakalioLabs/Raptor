import sys, os, math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.control.mpc_controller import MPCController, MPCConfig, MPCState

def test_mpc_zero_error():
    mpc = MPCController()
    state = MPCState(np.array([0,0,0]), np.array([0,0,0]))
    ref = [np.array([0,0,0])]
    u = mpc.compute_control(state, ref)
    assert np.allclose(u, 0, atol=0.1)

def test_mpc_tracks_target():
    mpc = MPCController()
    state = MPCState(np.array([0,0,10]), np.array([0,0,0]))
    ref = [np.array([10,0,10])]
    u = mpc.compute_control(state, ref)
    assert u[0] > 0

def test_mpc_prediction():
    mpc = MPCController()
    x0 = np.array([0,0,0,0,0,0])
    u_seq = [np.array([10,0,0]), np.array([10,0,0])]
    traj = mpc.predict_trajectory(x0, u_seq)
    assert traj.shape[0] == 3
    assert traj[-1][0] > 0

def test_mpc_clamped():
    cfg = MPCConfig(acceleration_max=1.0)
    mpc = MPCController(cfg)
    state = MPCState(np.array([0,0,0]), np.array([0,0,0]))
    ref = [np.array([1000,0,0])]
    u = mpc.compute_control(state, ref)
    assert abs(u[0]) <= 1.0 + 0.01
