import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.comm.network_monitor import NetworkMonitor, NetworkSample

def test_empty_monitor():
    nm = NetworkMonitor()
    assert nm.avg_latency == float('inf')
    assert nm.avg_bandwidth == 0.0
    assert not nm.link_healthy

def test_healthy_link():
    nm = NetworkMonitor()
    for _ in range(5):
        nm.add_sample(NetworkSample(0, 20, 50, 0.01, -70))
    assert nm.link_healthy
    assert abs(nm.avg_latency - 20.0) < 0.1

def test_unhealthy_loss():
    nm = NetworkMonitor(loss_threshold=0.1)
    for _ in range(5):
        nm.add_sample(NetworkSample(0, 20, 50, 0.5, -70))
    assert not nm.link_healthy

def test_signal_quality():
    nm = NetworkMonitor()
    nm.add_sample(NetworkSample(0, 10, 100, 0.0, -40))
    q = nm.get_signal_quality()
    assert q > 0.5
