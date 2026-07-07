import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.platform.task_offloader import TaskOffloader, TaskProfile, NetworkState, OffloadTarget

def test_latency_sensitive_local():
    to = TaskOffloader()
    task = TaskProfile("control", max_latency_ms=10, compute_flops=1e6, input_size_mb=0.1)
    net = NetworkState(rtt_mec_ms=20, rtt_cloud_ms=100, bandwidth_mbps=50, local_npu_utilization=0.5)
    assert to.decide(task, net) == OffloadTarget.LOCAL

def test_heavy_compute_mec():
    to = TaskOffloader()
    task = TaskProfile("detection", max_latency_ms=500, compute_flops=5e9, input_size_mb=1)
    net = NetworkState(rtt_mec_ms=10, rtt_cloud_ms=100, bandwidth_mbps=100, local_npu_utilization=0.3)
    assert to.decide(task, net) == OffloadTarget.MEC

def test_light_compute_local():
    to = TaskOffloader()
    task = TaskProfile("classify", max_latency_ms=200, compute_flops=1e8, input_size_mb=0.5)
    net = NetworkState(rtt_mec_ms=10, rtt_cloud_ms=100, bandwidth_mbps=100, local_npu_utilization=0.3)
    assert to.decide(task, net) == OffloadTarget.LOCAL

def test_npu_busy_offload():
    to = TaskOffloader()
    task = TaskProfile("detect", max_latency_ms=500, compute_flops=1e9, input_size_mb=0.5)
    net = NetworkState(rtt_mec_ms=10, rtt_cloud_ms=100, bandwidth_mbps=100, local_npu_utilization=0.95)
    assert to.decide(task, net) == OffloadTarget.MEC
