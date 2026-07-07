import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.slam.visual_inertial_odometry import EKFVIO, IMUMeasurement, VisualMeasurement

def test_initial_state():
    vio = EKFVIO()
    assert not vio.initialized
    assert np.allclose(vio.position, 0)

def test_predict_updates_position():
    vio = EKFVIO()
    imu = IMUMeasurement(accel=np.array([0, 0, 9.81]), gyro=np.zeros(3), dt=0.1)
    vio.predict(imu)
    assert vio.initialized
    assert np.allclose(vio.position, 0, atol=0.01)

def test_visual_update():
    vio = EKFVIO()
    imu = IMUMeasurement(accel=np.array([0,0,9.81]), gyro=np.zeros(3), dt=0.01)
    vio.predict(imu)
    vis = VisualMeasurement(feature_position=np.array([1,0,0]), confidence=0.8)
    vio.update_visual(vis)
    assert vio.position[0] > 0

def test_covariance_decreases():
    vio = EKFVIO()
    imu = IMUMeasurement(accel=np.array([0,0,9.81]), gyro=np.zeros(3), dt=0.01)
    vio.predict(imu)
    trace_before = vio.get_covariance_trace()
    vis = VisualMeasurement(feature_position=np.array([0,0,0]), confidence=1.0)
    vio.update_visual(vis)
    assert vio.get_covariance_trace() < trace_before
