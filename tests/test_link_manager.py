import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.comm.link_manager import LinkManager, LinkType, LinkQuality, QoSRequirement

def test_register_and_select():
    lm = LinkManager()
    lm.register_link(LinkType.LTE, LinkQuality(50, 10, 0.99, 0.01))
    lm.register_link(LinkType.NR5G, LinkQuality(5, 100, 0.999, 0.05))
    qos = QoSRequirement(max_latency_ms=100, min_bandwidth_mbps=5, min_reliability=0.9)
    best = lm.select_best_link(qos)
    assert best == LinkType.NR5G

def test_no_links():
    lm = LinkManager()
    qos = QoSRequirement(max_latency_ms=100, min_bandwidth_mbps=5, min_reliability=0.9)
    assert lm.select_best_link(qos) is None

def test_active_link():
    lm = LinkManager()
    lm.register_link(LinkType.LTE, LinkQuality(50, 10, 0.99, 0.01))
    assert lm.get_active_link() is None
