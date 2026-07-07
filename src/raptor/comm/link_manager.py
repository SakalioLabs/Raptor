"""Multi-link communication manager for 4G/5G/satellite links."""
from dataclasses import dataclass
from enum import Enum, auto


class LinkType(Enum):
    LTE = auto()
    NR5G = auto()
    SATELLITE = auto()
    MESH = auto()


@dataclass
class LinkQuality:
    latency_ms: float
    bandwidth_mbps: float
    reliability: float
    cost_per_mb: float


@dataclass
class QoSRequirement:
    max_latency_ms: float
    min_bandwidth_mbps: float
    min_reliability: float


@dataclass
class LinkState:
    link_type: LinkType
    quality: LinkQuality
    active: bool = False


class LinkManager:
    def __init__(self):
        self._links: dict[LinkType, LinkState] = {}

    def register_link(self, link_type: LinkType, quality: LinkQuality):
        self._links[link_type] = LinkState(link_type, quality)

    def update_quality(self, link_type: LinkType, quality: LinkQuality):
        if link_type in self._links:
            self._links[link_type].quality = quality

    def select_best_link(self, qos: QoSRequirement) -> LinkType | None:
        best_type, best_score = None, -1.0
        for lt, state in self._links.items():
            q = state.quality
            if q.latency_ms > qos.max_latency_ms * 2:
                continue
            if q.bandwidth_mbps < qos.min_bandwidth_mbps * 0.1:
                continue
            score = self._score_link(q, qos)
            if score > best_score:
                best_score = score
                best_type = lt
        return best_type

    def _score_link(self, q: LinkQuality, qos: QoSRequirement) -> float:
        lat = max(0, 1 - q.latency_ms / qos.max_latency_ms) * 40
        bw = min(1.0, q.bandwidth_mbps / qos.min_bandwidth_mbps) * 30
        rel = q.reliability * 20
        cost = max(0, 1 - q.cost_per_mb / 0.1) * 10
        return lat + bw + rel + cost

    def get_active_link(self) -> LinkType | None:
        for lt, state in self._links.items():
            if state.active:
                return lt
        return None
