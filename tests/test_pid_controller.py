import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.flight.pid_controller import PIDController, PIDGains

def test_proportional_only():
    pid = PIDController(PIDGains(kp=1.0, ki=0, kd=0))
    out = pid.update(0.5, 0.01)
    assert abs(out - 0.5) < 0.01

def test_output_clamp():
    pid = PIDController(PIDGains(kp=10.0, ki=0, kd=0), output_min=-1, output_max=1)
    out = pid.update(5.0, 0.01)
    assert out == 1.0

def test_zero_dt():
    pid = PIDController(PIDGains(kp=1.0))
    assert pid.update(1.0, 0.0) == 0.0

def test_integral_accumulation():
    pid = PIDController(PIDGains(kp=0, ki=1.0, kd=0))
    pid.update(1.0, 0.1)
    pid.update(1.0, 0.1)
    out = pid.update(1.0, 0.1)
    assert abs(out - 0.3) < 0.01

def test_reset():
    pid = PIDController(PIDGains(kp=0, ki=1.0, kd=0))
    pid.update(1.0, 0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error == 0.0

def test_derivative_action():
    pid = PIDController(PIDGains(kp=0, ki=0, kd=1.0), output_min=-100, output_max=100)
    pid.update(0.0, 0.1)
    out = pid.update(1.0, 0.1)
    assert abs(out - 10.0) < 0.1
