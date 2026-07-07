"""Network quality monitoring for UAV communication links."""
from dataclasses import dataclass, field
from collections import deque
import time


@dataclass
class NetworkSample:
    timestamp: float
    latency_ms: float
    bandwidth_mbps: float
    packet_loss: float
    rssi_dbm: float


class NetworkMonitor:
    def __init__(self, window_size: int = 50, loss_threshold: float = 0.1):
        self._samples: deque[NetworkSample] = deque(maxlen=window_size)
        self.loss_threshold = loss_threshold

    def add_sample(self, sample: NetworkSample):
        self._samples.append(sample)

    @property
    def avg_latency(self) -> float:
        if not self._samples:
            return float('inf')
        return sum(s.latency_ms for s in self._samples) / len(self._samples)

    @property
    def avg_bandwidth(self) -> float:
        if not self._samples:
            return 0.0
        return sum(s.bandwidth_mbps for s in self._samples) / len(self._samples)

    @property
    def avg_loss(self) -> float:
        if not self._samples:
            return 1.0
        return sum(s.packet_loss for s in self._samples) / len(self._samples)

    @property
    def link_healthy(self) -> bool:
        return self.avg_loss < self.loss_threshold and self.avg_latency < 500

    def get_signal_quality(self) -> float:
        if not self._samples:
            return 0.0
        latest = self._samples[-1]
        rssi_norm = max(0, min(1, (latest.rssi_dbm + 100) / 60))
        loss_score = max(0, 1 - latest.packet_loss / 0.5)
        lat_score = max(0, 1 - latest.latency_ms / 200)
        return (rssi_norm * 0.4 + loss_score * 0.3 + lat_score * 0.3)
