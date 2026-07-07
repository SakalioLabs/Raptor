"""Edge-cloud task offloading decision engine."""
from dataclasses import dataclass
from enum import Enum, auto


class OffloadTarget(Enum):
    LOCAL = auto()
    MEC = auto()
    CLOUD = auto()


@dataclass
class TaskProfile:
    name: str
    max_latency_ms: float
    compute_flops: float
    input_size_mb: float


@dataclass
class NetworkState:
    rtt_mec_ms: float
    rtt_cloud_ms: float
    bandwidth_mbps: float
    local_npu_utilization: float


class TaskOffloader:
    def __init__(self, local_flops: float = 1e9, local_threshold: float = 0.8):
        self.local_flops = local_flops
        self.local_threshold = local_threshold

    def decide(self, task: TaskProfile, net: NetworkState) -> OffloadTarget:
        if task.max_latency_ms < 50:
            return OffloadTarget.LOCAL
        if net.local_npu_utilization > self.local_threshold:
            if task.max_latency_ms > net.rtt_mec_ms + 10:
                return OffloadTarget.MEC
            return OffloadTarget.LOCAL
        transfer_ms = (task.input_size_mb * 8) / max(0.1, net.bandwidth_mbps) * 1000
        total_mec = net.rtt_mec_ms + transfer_ms
        total_cloud = net.rtt_cloud_ms + transfer_ms
        if task.compute_flops > self.local_flops and total_mec < task.max_latency_ms:
            return OffloadTarget.MEC
        if task.compute_flops > self.local_flops * 5 and total_cloud < task.max_latency_ms:
            return OffloadTarget.CLOUD
        return OffloadTarget.LOCAL
